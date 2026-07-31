#2026-07-30
# navi_factory.sdf의 물리 스텝(max_step_size)을 늘려 real-time factor 문제를
# 고친 뒤로는 DiffDrive가 정상 동작하므로, cmd_vel 기반 웨이포인트 추종으로 되돌렸다.
#
# odom(바퀴 적산치)은 슬립이 있으면 실제 위치와 어긋나서, 대신 gz-sim-pose-publisher-system이
# 발행하는 진짜(ground-truth) 월드 좌표(pose_gt, bridge.yaml 참고)를 직접 구독한다.
#
# 코너에서 고속으로 방향을 홱 트는 하드 컷오프(각도 넘으면 속도 0)가 전복을
# 유발해서, heading_error 크기에 비례해 부드럽게 감속하는 방식으로 바꿨다.

import math
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node

# 코너를 웨이포인트 하나(0,13)로 90도 홱 꺾게 했더니 실제 트랙 라인을 벗어나
# 크게 돌아나갔다 — track_ne_entry_7(0,14.75)/entry_8(1.25,13) 타일 좌표를 보고
# 코너를 두 단계로 나눠, 실제 타일이 깔린 자리 위로만 지나가게 한다.
WAYPOINTS = [
    (0.0, 32.25),   # 출발 (entry 북쪽 끝)
    (0.0, 14.0),    # 코너 진입 직전 — 아직 수직 트랙(entry_7) 위
    (1.0, 13.0),    # 코너 중간 — 수평 트랙(entry_8) 시작 지점 위로 대각 전환
    (8.75, 13.0),   # 도착 (entry 동쪽 끝)
]

# 웨이포인트 간격이 좁아진 코너 구간(약 1.5~2m)에 맞춰 도착 판정도 더 빡빡하게.
ARRIVAL_TOLERANCE = 0.6
# 코너를 못 따라가고 홱홱 튀는 문제 → 순항 속도 자체를 1.5배 낮춤.
LINEAR_SPEED = 5.0 / 1.5  # ≈3.33
# P만으로도, PD(게인 높임)로도 heading_error가 한쪽으로 계속 헌팅하며 안 멈췄다.
# 반응 지연(제어 주기가 실제 물리 반응보다 빠름) 때문에 미분항이 지연된/노이즈 낀
# 신호를 증폭시켜 오히려 안 좋았던 것으로 보인다 — 게인을 낮추고 미분 비중도
# 줄이고, 무엇보다 제어 주기를 훨씬 느긋하게 늘려 반응이 실제로 나타날 시간을 준다.
ANGULAR_GAIN = 0.8        # P
ANGULAR_DAMPING = 0.15    # D — 최소한만 사용(노이즈 증폭 방지)
MAX_ANGULAR = 1.2
# heading_error가 이 각도 이상이면 완전 정지(제자리 회전), 0이면 전속력 —
# 그 사이는 선형 보간으로 부드럽게 감속(전복 방지). dead-band(SLOWDOWN_START_ANGLE)를
# 넓혀서 약간의 각도 오차는 그냥 직진하며 자연스럽게 보정하게 한다.
FULL_STOP_ANGLE = 1.2     # rad (~69도) 이상이면 완전 정지 후 회전
SLOWDOWN_START_ANGLE = 0.5  # rad (~29도) 부터 감속 시작

# 코너(다음 웨이포인트)에 가까워질수록 속도를 줄여서 고속으로 코너를 오버슈트
# 하며 헌팅하는 걸 막는다 — 헤딩 오차 기반 감속과는 별개로, 거리 기반으로도 감속.
#
# 제어 주기가 1.2초라 도착 판정(ARRIVAL_TOLERANCE) 시점의 속도로 최대 1.2초를
# "관성 주행"한 뒤에야 다음 목표로 방향을 튼다 — 실측 결과 이 관성 주행 거리가
# 1.5~2m나 돼서 코너를 한참 지나친 뒤에야 도는 문제가 있었다. 감속을 훨씬 더
# 일찍(6m 전) 시작하고 최저 속도도 훨씬 낮춰서, 도착 판정 시점의 실제 속도 자체를
# 낮게 유지한다 → 관성 주행 거리(속도 x 1.2초)가 짧아져 코너를 거의 지나치지 않는다.
CORNER_SLOWDOWN_DISTANCE = 6.0    # 이 거리 안부터 감속 시작
MIN_DISTANCE_SPEED_FACTOR = 0.15  # 코너 근처 최소 속도 배율(조향은 계속 가능하게 완전정지는 아님)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def speed_factor(abs_heading_error):
    """heading_error가 커질수록 0~1 사이로 부드럽게 줄어드는 배율."""
    if abs_heading_error <= SLOWDOWN_START_ANGLE:
        return 1.0
    if abs_heading_error >= FULL_STOP_ANGLE:
        return 0.0
    span = FULL_STOP_ANGLE - SLOWDOWN_START_ANGLE
    return 1.0 - (abs_heading_error - SLOWDOWN_START_ANGLE) / span


def distance_speed_factor(distance):
    """다음 웨이포인트(코너 포함)에 가까워질수록 1.0~MIN_DISTANCE_SPEED_FACTOR로 감속."""
    if distance >= CORNER_SLOWDOWN_DISTANCE:
        return 1.0
    if distance <= 0.0:
        return MIN_DISTANCE_SPEED_FACTOR
    ratio = distance / CORNER_SLOWDOWN_DISTANCE
    return MIN_DISTANCE_SPEED_FACTOR + (1.0 - MIN_DISTANCE_SPEED_FACTOR) * ratio


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._index = 0
        self._dir = 1
        self._pose = None
        self._leg_start = time.monotonic()
        self._tick_count = 0
        self._prev_heading_error = None
        self._prev_time = None
        self.create_subscription(Pose, 'pose_gt', self._on_pose, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        # 0.3s도 여전히 물리 반응(실시간 배속 저하 + 관성)보다 빨라서 헌팅이
        # 계속됐다 — 훨씬 느긋하게(1.2s) 줘서 이전 명령의 결과가 실제로 나타난
        # 뒤에 다음 보정을 하게 한다.
        self.create_timer(1.2, self._on_timer)

    def _on_pose(self, msg):
        self._pose = msg

    def _advance_waypoint(self):
        elapsed = time.monotonic() - self._leg_start
        self.get_logger().info(f'구간 소요시간(실제 초): {elapsed:.1f}s')
        self._leg_start = time.monotonic()
        n = len(WAYPOINTS)
        if n < 2:
            return
        self._index += self._dir
        if self._index >= n:
            self._index = n - 2
            self._dir = -1
        elif self._index < 0:
            self._index = 1
            self._dir = 1

    def _on_timer(self):
        if self._pose is None:
            return

        x, y = self._pose.position.x, self._pose.position.y
        yaw = yaw_from_quaternion(self._pose.orientation)
        tx, ty = WAYPOINTS[self._index]
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)

        if distance < ARRIVAL_TOLERANCE:
            self.get_logger().info(f'도착: waypoint[{self._index}] ({tx:.2f},{ty:.2f})')
            self._advance_waypoint()
            return

        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)

        now = time.monotonic()
        derivative = 0.0
        if self._prev_heading_error is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 1e-3:
                derivative = normalize_angle(heading_error - self._prev_heading_error) / dt
        self._prev_heading_error = heading_error
        self._prev_time = now

        cmd = Twist()
        pd = ANGULAR_GAIN * heading_error + ANGULAR_DAMPING * derivative
        cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, pd))
        cmd.linear.x = (LINEAR_SPEED * speed_factor(abs(heading_error))
                         * distance_speed_factor(distance))
        self._pub.publish(cmd)

        self._tick_count += 1
        if self._tick_count % 5 == 0:  # 대략 1.5초(실제 시간)마다 진단 로그
            self.get_logger().info(
                f'diag pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg '
                f'target=({tx:.2f},{ty:.2f}) dist={distance:.2f} '
                f'heading_err={math.degrees(heading_error):.1f}deg '
                f'cmd=(lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f})'
            )


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
