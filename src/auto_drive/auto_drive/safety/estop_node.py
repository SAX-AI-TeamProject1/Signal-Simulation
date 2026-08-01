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

from collections import deque
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

        # 광선 하나만 튀는 노이즈로는 서지 않는다. 다만 "통로 전체에서 2발"로 세면
        # 서로 무관한 단발 두 개(왼쪽 끝 기둥 모서리 + 오른쪽 끝 먼지)에도 서 버린다.
        # 그래서 이웃한 cluster_window 발 안에 min_hits 발이 몰렸을 때만 물체로 본다.
        # 통로 길이는 길어야 clear_distance(6 m)이고 각 간격이 0.25도이므로, 그 끝에서도
        # 사람 다리 하나에 5.7발이 맞는다 — 가랑이 틈에 다리가 둘로 갈려도 각각이
        # 문턱을 넘으므로 틈을 이어 붙일 필요가 없다.
        # 대가는 6 m 앞의 폭 5.2 cm 미만 물체를 놓치는 것이다(2발을 못 받는다).
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('cluster_window', 7)

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

        # vision-signal과 다른 이유는, gz에서 오는 라이다 정보를 받아 오기 때문
        # vision은 실제 웹캠으로 부터 노드가 직접 읽어오기 때문에, 구현의 차이가 존재합니다.
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
        # 히스테리시스: 과거의 영향; 현재의 값이나 출력이 오직 지금의 조건만으로 결정되지 않고, 거쳐 온 경로에 의존합니다.
        limit = clear_distance if self._stopping else stop_distance
        window = max(1, self.get_parameter('cluster_window').value)
        # 1차: 광선마다 통로 안인지 표시만 한다(배열 길이는 원본 그대로).
        # 2차: 그 배열 위로 창을 밀어 뭉쳐 있는 것만 남긴다.
        rays = self.rays_in_corridor(msg, half_width, limit)
        hits, nearest = self.cluster_hits(rays, window, min_hits)

        if hits > 0:
            self._begin_stop(f'통로 안 {hits}개 반사, 최근접 {nearest:.2f} m')
        else:
            self._try_release()

    # 차체가 가려는 길(통로)에 걸린 ray 모으기
    @staticmethod
    def rays_in_corridor(msg, half_width, limit):
        """
        진행 통로(폭 2*half_width, 길이 limit) 안에 들어온 반사들.

        msg 에서 읽는 필드:
          angle_min, angle_max : 스캔 시작/끝 각도
          angle_increment : 스캔 각도 간격
          ranges[] : 각도별 거리 값 배열
          range_min, range_max : 측정 가능 최소/최대 거리

        ranges 와 길이도 순서도 같은 (유효, 거리) 배열을 돌려준다. 걸러진 광선도
        자리를 비워 두고 남긴다 — 통과한 것만 모아 붙이면 원래 떨어져 있던 반사가
        이웃이 되어 버려서, 뒤에서 cluster_hits 가 창을 밀 축이 사라진다.

        좌표는 스캔 프레임(lidar_link) 그대로 쓴다. base_link 와 회전이 같고
        (urdf 의 lidar_joint 가 rpy 0), 통로를 라이다 기준으로 재는 게 곧 차체
        앞쪽을 재는 것이다 — 라이다가 차체 앞끝보다 0.15 m 더 앞에 있다.
        """
        rays = [(False, 0.0)] * len(msg.ranges)
        for index, distance in enumerate(msg.ranges):
            # 수치적으로 존재하는 값인지
            if not math.isfinite(distance):
                continue
            # 데이터 필터(내가 정한 거리 인지)
            if distance < msg.range_min or distance > msg.range_max:
                continue

            # 인자로 들어온 거리로, 이 거리 내에 있어야 hit판정
            # stop상태면, clear_dis, !stop이면 stop_dis
            if distance > limit:
                # 통로 길이를 넘었다. 각도와 무관하게 볼 필요가 없다.
                continue

            angle = msg.angle_min + index * msg.angle_increment
            # 옆이나 뒤로 잡힌 반사는 갈 길이 아니다.
            # 차체의 룩 베터 기준 -90~90이기떄문에 => 0<=cos(-90~90)<=1
            if math.cos(angle) <= 0.0:
                continue

            # hit ray를 dis*sin(angle)하면
            # 차체의 폭과 평행한 위치의 거리?를 알 수 있다.
            # (라이다가 차체 머리쪽에 휘어있지 않게 달려 있어서 가능)
            # \| -> \가 dis이고 사이각이 angle이면 dis*sin(angle)하면
            # - 의 길이를 알 수 있기 때문

            # 차폭이 통과 가능한 거리에 있다면 넘어간다
            if abs(distance * math.sin(angle)) > half_width:
                continue

            rays[index] = (True, distance)
        return rays

    @staticmethod
    def cluster_hits(rays, window, min_hits):
        """
        연속한 광선 window 칸 안에 유효 반사가 min_hits 개 이상인 구간만 남긴다.

        남은 반사 수와 그중 최근접 거리를 돌려준다. 뭉치지 않은 단발은 버리므로,
        통로 양 끝에서 하나씩 튄 무관한 반사 두 개로는 정지가 걸리지 않는다.
        진짜 물체는 광선이 붙어서 돌아오므로 이 조건에 걸리지 않는다.

        창은 원본 스캔 배열 위를 그대로 민다. 1차에서 걸러진 칸도 자리를 차지하니
        광선 사이의 실제 간격이 그대로 반영된다. 창 하나가 문턱을 넘으면 그 창
        안의 유효 반사를 전부 남기므로, 창이 겹치면서 긴 덩어리는 통째로 남는다.

        창을 옮길 때마다 7칸을 다시 세지 않고, 들어온 것 하나를 넣고 뒤로 밀려난
        것만 버린다. deque 를 쓰는 이유는 양쪽 끝 넣고 빼기가 O(1) 이면서 C 구현이라
        루프 본문이 인터프리터를 덜 타기 때문이다 — queue.Queue 는 스레드용 락을
        들고 있어 이런 용도에는 맞지 않는다.
        """
        keep = set()
        run = deque()  # 지금 창 안에 살아 있는 유효 광선들의 인덱스
        for index, (valid, dis) in enumerate(rays):
            if valid:
                run.append(index)
            while run and run[0] <= index - window:  # 창 뒤로 빠져나간 것
                run.popleft()
            # index 가 창 하나를 채우기 전까지는 아직 판정할 창이 없다.
            if index >= window - 1 and len(run) >= min_hits:
                keep.update(run)  # batch insert, param은 iterable

        # if len(keep) == 0:
        if not keep:
            return 0, float('inf')

        # 유효한 대상중에 길이가 가장 짧은 거리 리턴
        return len(keep), min(rays[i][1] for i in keep)

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
