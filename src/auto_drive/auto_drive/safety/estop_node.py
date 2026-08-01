"""
라이다 스캔을 보고 진행 경로에 장애물이 있으면 로봇을 세운다.

용도는 자율 회피가 아니다. 로봇은 정해진 트랙을 가고, 그 길 위로 사람이 자주
지나다니는 상황에서 "앞에 뭔가 있으면 일단 선다"만 한다. 비켜서 지나가는 판단은
하지 않는다 — 사람이 지나가면 다시 출발한다.

판단 영역은 부채꼴이 아니라 로봇 폭만큼의 직사각형 통로다. 부채꼴로 잡으면 길
옆에 서 있는 선반이나 통로 밖으로 지나가는 사람에도 서 버려서, 정작 필요한
순간에 "또 오작동이겠지"가 된다. 갈 길 위에 있는 것만 본다.

twist_mux 로 내보내는 방식: cmd_vel_estop 은 우선순위 100 으로 최상위이고
timeout 이 0.3초다(config/twist_mux.yaml). 그래서 "정지 해제"라는 신호를 따로
보낼 필요가 없다 — 멈춰야 하는 동안만 0 속도를 계속 쏘고, 길이 트이면 발행을
멈추면 0.3초 뒤에 twist_mux 가 알아서 다음 우선순위(수신호·teleop·순찰)를
통과시킨다.

거리 값의 근거(실측 파라미터로 계산):
  순항 3.33 m/s (waypoint_follower 의 LINEAR_SPEED)
  감속 1.5 m/s^2 (urdf DiffDrive 의 max_linear_acceleration)
  제동 거리 v^2/2a = 3.70 m
  반응 거리 = 3.33 x (스캔 100ms + 처리 50ms) = 0.50 m
  합 4.20 m  → 정지 5.0 m 로 잡아 0.8 m 여유를 둔다.

이 노드가 보는 것은 2D 단면 한 장뿐이다. 라이다는 지면 0.35 m 높이에 있으므로
그보다 낮은 팔레트나, 상판이 머리 위로 튀어나온 물체는 보이지 않는다. 이 한계는
doc/design.md 에 적혀 있고, 여기서 해결할 수 있는 문제가 아니다.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class EstopNode(Node):
    """진행 통로 안의 장애물을 보고 cmd_vel_estop 을 발행한다."""

    def __init__(self):
        super().__init__('estop_node')

        # 정지/해제 거리를 다르게 두는 이유: 같은 값이면 장애물이 경계에 걸쳐 있을 때
        # 정지와 해제를 빠르게 반복해서 로봇이 덜컥거린다.
        self.declare_parameter('stop_distance', 5.0)
        self.declare_parameter('clear_distance', 6.0)
        # 차체 폭 1.09 m 의 반폭 0.545 m 에 여유를 더한 값. 통로 폭 1.4 m.
        self.declare_parameter('half_width', 0.7)
        # 광선 하나만 튀는 노이즈로는 서지 않는다. 폭 0.4 m 인 사람은 0.25도 간격에서
        # 5 m 거리에 18개, 20 m 에서도 4개가 걸리므로 2개 요구는 사람을 놓치지 않는다.
        self.declare_parameter('min_hits', 2)
        # 사람이 경계에서 머뭇거릴 때 곧바로 재출발하지 않도록 하는 최소 정지 시간.
        self.declare_parameter('min_hold_sec', 0.5)
        # 스캔이 끊기면 선다. 눈을 잃은 채로 계속 가는 것보다 서는 쪽이 안전하다.
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('stop_on_scan_timeout', True)
        # 스캔(10Hz)보다 빠르게 쏜다. twist_mux 의 timeout 0.3초 안에 반드시 다음
        # 메시지가 들어가야 정지가 풀리지 않는다.
        self.declare_parameter('publish_rate', 20.0)

        self._stopping = False
        self._stop_started = None
        self._last_scan_time = None
        self._last_reason = ''

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_estop', 1)
        # 왜 멈췄는지 밖에서 보려고 상태도 낸다. 주행에는 쓰이지 않는다.
        self._state_pub = self.create_publisher(Bool, 'estop', 1)

        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        rate = max(1.0, self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'정지 구간 {self.get_parameter("stop_distance").value} m, '
            f'해제 {self.get_parameter("clear_distance").value} m, '
            f'통로 반폭 {self.get_parameter("half_width").value} m')

    def _on_scan(self, msg):
        self._last_scan_time = self.get_clock().now()

        half_width = self.get_parameter('half_width').value
        stop_distance = self.get_parameter('stop_distance').value
        clear_distance = self.get_parameter('clear_distance').value
        min_hits = max(1, self.get_parameter('min_hits').value)

        # 정지 중에는 더 먼 거리까지 비어야 풀어 준다(히스테리시스).
        limit = clear_distance if self._stopping else stop_distance
        hits, nearest = self.count_in_corridor(msg, half_width, limit)

        if hits >= min_hits:
            self._begin_stop(f'통로 안 {hits}개 반사, 최근접 {nearest:.2f} m')
        else:
            self._try_release()

    @staticmethod
    def count_in_corridor(msg, half_width, limit):
        """
        진행 통로(폭 2*half_width, 길이 limit) 안에 들어온 반사 수와 최근접 거리.

        좌표는 스캔 프레임(lidar_link) 그대로 쓴다. base_link 와 회전이 같고
        (urdf 의 lidar_joint 가 rpy 0), 통로를 라이다 기준으로 재는 게 곧 차체
        앞쪽을 재는 것이다 — 라이다가 차체 앞끝보다 0.15 m 더 앞에 있다.
        """
        hits = 0
        nearest = float('inf')
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if distance > limit:
                # 통로 길이를 넘었다. 각도와 무관하게 볼 필요가 없다.
                continue
            angle = msg.angle_min + index * msg.angle_increment
            # 옆이나 뒤로 잡힌 반사는 갈 길이 아니다.
            if math.cos(angle) <= 0.0:
                continue
            if abs(distance * math.sin(angle)) > half_width:
                continue
            hits += 1
            nearest = min(nearest, distance)
        return hits, nearest

    def _begin_stop(self, reason):
        if not self._stopping:
            self._stopping = True
            self._stop_started = self.get_clock().now()
            self.get_logger().warn(f'정지: {reason}')
        self._last_reason = reason

    def _try_release(self):
        if not self._stopping:
            return
        held = (self.get_clock().now() - self._stop_started).nanoseconds * 1e-9
        if held < self.get_parameter('min_hold_sec').value:
            return
        self._stopping = False
        self._stop_started = None
        self.get_logger().info(f'해제: 통로가 비었다 ({held:.1f}초 정지)')

    def _scan_is_stale(self):
        if not self.get_parameter('stop_on_scan_timeout').value:
            return False
        if self._last_scan_time is None:
            # 아직 스캔이 한 번도 안 왔다. 볼 수 없으면 가지 않는다.
            return True
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds * 1e-9
        return age > self.get_parameter('scan_timeout_sec').value

    def _on_timer(self):
        stale = self._scan_is_stale()
        if stale and not self._stopping:
            self._begin_stop('스캔이 끊겼다')

        stopping = self._stopping or stale

        state = Bool()
        state.data = stopping
        self._state_pub.publish(state)

        if stopping:
            # 0 속도를 계속 쏘는 동안만 정지가 유지된다. 풀 때는 그냥 멈추면 된다 —
            # twist_mux 가 timeout 으로 알아서 다음 소스를 통과시킨다.
            self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT 는 KeyboardInterrupt, SIGTERM 은 ExternalShutdownException 으로 온다.
        # 뒤엣것을 안 잡으면 launch 가 노드를 내릴 때 역추적이 통째로 찍힌다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
