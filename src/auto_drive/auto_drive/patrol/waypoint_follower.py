# 2026-08-01
# 물리 기반(cmd_vel → DiffDrive → 바퀴-바닥 접촉) 웨이포인트 추종에서 '스톨' 버그가
# 있었다 — 바퀴 관절은 명령대로 정상 회전하는데(joint_states로 확인) 차체가 실제로는
# 접지력을 못 받아 제자리에 멈추는 현상. 원인을 다시 분석한 결과, 바퀴 바닥
# (chassis 기준 -0.01)과 캐스터 바닥(+0.02)이 3cm 어긋나 있어서 평상시 순항 중엔
# 캐스터가 바닥에서 떠 있다가 급가감속으로 섀시가 피치할 때만 갑자기 접촉하는
# 구조였다 — 이 "상대속도가 있는 상태의 갑작스런 신규 접촉"이 DART 접촉 솔버를
# 불안정하게 만든 것으로 보인다. mecanum_lift_robot.urdf.xacro에서 캐스터 높이를
# 바퀴와 같은 깊이로 맞추고(4점이 스폰 직후부터 동시에 자리잡음), 접촉 강성/감쇠
# (kp/kd/min_depth)를 명시해 충격성 접촉이 한 스텝에 힘을 몰아주지 않게 했다.
#
# odom(바퀴 적산치)은 슬립이 있으면 실제 위치와 어긋나서, 대신 gz-sim-pose-publisher-system이
# 발행하는 진짜(ground-truth) 월드 좌표(pose_gt, bridge.yaml 참고)를 직접 구독한다.

import math
from pathlib import Path
import time

from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.node import Node
import yaml

# navi_factory.sdf의 track_* 타일 pose 실측 결과(같은 폴더의 tracks.yaml 상단 주석 참고):
# 북쪽 공용 허브(신호지점 근처)에 반원 회차루프(signal_u, 중심(0,40) 반지름 3m)가 있고,
# 그 양 끝이 nw_hub_merge/se_hub_merge를 통해 각각 x=-2/x=2 간선에 연결된다. x=0
# 간선("1차선")은 허브 루프에 직접 붙은 타일이 없어서, 루프를 타려면 허브 안쪽(빈
# 바닥)에서 x=2쪽으로 살짝 붙었다 다시 돌아와야 한다. 지금은 1차선만 왕복시킨다:
# 출발(신호지점) → 회차루프 한 바퀴 → 1차선 진입 → 남쪽 끝(도착) → 왕복.
WAYPOINTS = [
    (0.0, 36.05),        # 출발(신호지점) — track_signal_entry_2
    (2.0, 37.25),        # 허브 안, x=2(se_trunk) 쪽으로 붙음(타일 없는 구간)
    (2.5, 38.625),       # se_hub_merge
    (3.0, 40.0),         # 회차루프 진입 — signal_u_1(동쪽 끝)
    (2.42705, 41.7634),  # 회차루프 — signal_u_2
    (0.927051, 42.8532),  # 회차루프 — signal_u_3
    (-0.927051, 42.8532),  # 회차루프 — signal_u_4
    (-2.42705, 41.7634),  # 회차루프 — signal_u_5
    (-3.0, 40.0),        # 회차루프 진출 — signal_u_6(서쪽 끝)
    (-2.5, 38.625),      # nw_hub_merge
    (-2.0, 37.25),       # 허브 안, x=0(1차선)으로 복귀(타일 없는 구간)
    (0.0, 37.25),        # 1차선(ne_entry) 진입
    (0.0, -12.1),        # 도착 — track_ne_entry_south_11(1차선 남쪽 끝)
]

# 0.3s도 여전히 물리 반응(실시간 배속 저하 + 관성)보다 빨라서 헌팅이 계속됐다 —
# 훨씬 느긋하게(1.2s) 줘서 이전 명령의 결과가 실제로 나타난 뒤에 다음 보정을 하게 한다.
CONTROL_PERIOD_SEC = 1.2

ARRIVAL_TOLERANCE = 0.6
# 코너를 못 따라가고 홱홱 튀는 문제 → 순항 속도 자체를 1.5배 낮춤.
LINEAR_SPEED = 5.0 / 1.5  # ≈3.33
ANGULAR_GAIN = 0.8        # P
ANGULAR_DAMPING = 0.15    # D — 최소한만 사용(노이즈 증폭 방지)
MAX_ANGULAR = 1.2
# heading_error가 이 각도 이상이면 완전 정지(제자리 회전), 0이면 전속력 —
# 그 사이는 선형 보간으로 부드럽게 감속(전복 방지).
FULL_STOP_ANGLE = 1.2     # rad (~69도) 이상이면 완전 정지 후 회전
SLOWDOWN_START_ANGLE = 0.5  # rad (~29도) 부터 감속 시작

# 목표 각속도가 한 tick 만에 크게 튀면 실제 로봇이 매 tick(1.2초)마다 방향을 홱홱
# 반대로 트는 "휘청거림"으로 나타난다. 실제 발행값은 직전 tick 대비 이만큼만 변하도록
# 제한해서 방향 반전이 여러 tick에 걸쳐 부드럽게 일어나게 한다.
MAX_ANGULAR_STEP = 0.5  # rad/s — tick당 각속도 변화 상한

# 코너(다음 웨이포인트)에 가까워질수록 속도를 줄여서 고속으로 코너를 오버슈트
# 하며 헌팅하는 걸 막는다 — 헤딩 오차 기반 감속과는 별개로, 거리 기반으로도 감속.
CORNER_SLOWDOWN_DISTANCE = 6.0    # 이 거리 안부터 감속 시작
MIN_DISTANCE_SPEED_FACTOR = 0.15  # 코너 근처 최소 속도 배율(조향은 계속 가능하게 완전정지는 아님)

# 직전→목표 웨이포인트를 잇는 트랙 선분 위에서 "현재 위치의 투영점보다 조금 앞"을
# 조준점으로 삼는 pure-pursuit 방식 — 목표점을 직접 조준하면 목표가 멀수록 직선
# 트랙에서 크게 벗어난 대각선으로 질러가게 된다. 조준점은 최근 명령 속도 x 제어
# 주기에 맞춰 동적으로 정한다(순항 중엔 멀리, 코너 근처처럼 느릴 땐 가깝게).
MIN_LOOKAHEAD_DISTANCE = 1.0    # m — 정지에 가까울 때(코너 근처)의 최소 조준 거리
LOOKAHEAD_TIME_FACTOR = 1.5     # 제어 주기 대비 몇 배 앞을 조준할지

# WAYPOINTS의 각 점이 코너/목적지(station)/신호 대기 지점(signal_point) 중 무엇인지는
# tracks.yaml의 signal_point/stations 좌표와 거리 대조해서 정한다(아래
# classify_waypoints 참고) — 두 목록이 같은 물리 지점을 가리키지만, WAYPOINTS는 코너
# 완화를 위해 세분화된 실주행 경로라 인덱스를 그대로 맞출 수 없기 때문이다.
POINT_TYPE_TOLERANCE = 0.5  # m — 이 거리 이내면 같은 지점으로 본다
SIGNAL_DWELL_SEC = 3.0      # 신호 대기 지점 도착 시 정지하고 기다리는 시간
STATION_DWELL_SEC = 2.0     # 목적지 도착 시 정차하는 시간


def load_track_points(tracks_yaml_path):
    """tracks.yaml에서 signal_point(단일 좌표)/stations(이름→좌표) 목록을 읽는다."""
    with open(tracks_yaml_path) as f:
        data = yaml.safe_load(f) or {}
    signal_point = data.get('signal_point')
    stations = list((data.get('stations') or {}).values())
    return signal_point, stations


def classify_waypoints(waypoints, signal_point, stations, tolerance=POINT_TYPE_TOLERANCE):
    """각 웨이포인트를 'signal_point' / 'station' / 'corner' 중 하나로 분류해 반환."""
    def _matches(point, candidates):
        return any(math.hypot(point[0] - c[0], point[1] - c[1]) <= tolerance
                   for c in candidates)

    types = []
    for wp in waypoints:
        if signal_point is not None and _matches(wp, [signal_point]):
            types.append('signal_point')
        elif stations and _matches(wp, stations):
            types.append('station')
        else:
            types.append('corner')
    return types


def lookahead_point(prev, target, position, lookahead_dist):
    """
    직전→목표 웨이포인트 선분 위에서 조준점을 계산한다.

    현재 위치의 투영점보다 lookahead_dist만큼 목표 쪽으로 나아간 점을 반환한다
    (트랙 선분을 벗어나 있으면 그 선으로 끌어당기는 효과). 투영점은 선분 밖으로
    나가지 않게 [0, 선분 길이]로 clamp한다.
    """
    px, py = prev
    tx, ty = target
    x, y = position
    seg_dx, seg_dy = tx - px, ty - py
    seg_len = math.hypot(seg_dx, seg_dy)
    if seg_len < 1e-6:
        return target

    ux, uy = seg_dx / seg_len, seg_dy / seg_len
    progress = (x - px) * ux + (y - py) * uy
    progress = max(0.0, min(seg_len, progress))
    look = min(seg_len, progress + lookahead_dist)
    return px + ux * look, py + uy * look


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

        self.declare_parameter('tracks_yaml_path', '')
        self.declare_parameter('signal_dwell_sec', SIGNAL_DWELL_SEC)
        self.declare_parameter('station_dwell_sec', STATION_DWELL_SEC)

        tracks_yaml_path = self.get_parameter('tracks_yaml_path').value
        signal_point, stations = None, []
        if tracks_yaml_path:
            if not Path(tracks_yaml_path).is_file():
                raise RuntimeError(
                    f'tracks_yaml_path가 지정됐지만 파일을 찾을 수 없습니다: '
                    f"'{tracks_yaml_path}'")
            signal_point, stations = load_track_points(tracks_yaml_path)
        else:
            self.get_logger().warn(
                'tracks_yaml_path 파라미터가 비어 있어 모든 웨이포인트를 코너로 취급합니다 '
                '(신호 대기/목적지 정차 없이 예전처럼 곧바로 되돌아감). launch에서 '
                'worlds/navi_factory/world/navi_factory/tracks.yaml 경로를 '
                '넘기면 활성화됩니다.')

        self._waypoint_types = classify_waypoints(WAYPOINTS, signal_point, stations)
        self.get_logger().info(
            'waypoint 타입 분류: '
            + ', '.join(f'[{i}]{t}' for i, t in enumerate(self._waypoint_types)))

        self._signal_dwell_sec = self.get_parameter('signal_dwell_sec').value
        self._station_dwell_sec = self.get_parameter('station_dwell_sec').value

        self._index = 0
        self._dir = 1
        self._pose = None
        self._leg_start = time.monotonic()
        self._tick_count = 0
        self._prev_heading_error = None
        self._prev_time = None
        self._dwell_until = None  # None이면 주행 중, 값이 있으면 그 시각까지 정지 유지
        self._last_linear_speed = 0.0  # 조준점 lookahead 거리를 speed에 맞춰 조정하는 데 사용
        self._last_angular = 0.0  # 각속도 slew-rate 제한(MAX_ANGULAR_STEP)의 기준값
        self.create_subscription(Pose, 'pose_gt', self._on_pose, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.create_timer(CONTROL_PERIOD_SEC, self._on_timer)

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
        # 목표가 바뀌면 heading_error가 코너 각도만큼 한 틱만에 점프한다 — 이전
        # 목표 기준의 heading_error를 기준값으로 그대로 두면 미분항이 그 점프를
        # "변화량"으로 잡아 한 틱은 과하게, 다음 틱은 반대로 보정하는 "derivative
        # kick"이 생긴다(PID의 잘 알려진 함정). 새 목표의 첫 틱은 미분항 없이
        # P만으로 반응하도록 리셋한다.
        self._prev_heading_error = None
        self._prev_time = None

    def _on_arrival(self):
        """
        웨이포인트 도착 처리. 타입(코너/목적지/신호 대기 지점)에 따라 다르게 반응한다.

        코너는 곧바로 다음 목표로 넘어간다. 목적지/신호 대기 지점은 정지한 뒤 일정
        시간 대기하고서 넘어간다 — 신호 대기 지점에서는 twist_mux가 gesture(50)를
        auto(10)보다 우선 통과시키므로, 정지해 있는 동안 수신호가 들어오면 자연스럽게
        그쪽으로 제어권이 넘어간다(별도 핸드오프 로직 불필요).
        """
        wp_type = self._waypoint_types[self._index]
        tx, ty = WAYPOINTS[self._index]
        if wp_type == 'signal_point':
            self.get_logger().info(
                f'신호 대기 지점 도착: waypoint[{self._index}] ({tx:.2f},{ty:.2f}) — '
                f'{self._signal_dwell_sec:.1f}초 정지')
            self._pub.publish(Twist())
            self._last_linear_speed = 0.0
            self._last_angular = 0.0
            self._dwell_until = time.monotonic() + self._signal_dwell_sec
            return
        if wp_type == 'station':
            self.get_logger().info(
                f'목적지 도착: waypoint[{self._index}] ({tx:.2f},{ty:.2f}) — '
                f'{self._station_dwell_sec:.1f}초 정차')
            self._pub.publish(Twist())
            self._last_linear_speed = 0.0
            self._last_angular = 0.0
            self._dwell_until = time.monotonic() + self._station_dwell_sec
            return
        self.get_logger().info(f'코너 통과: waypoint[{self._index}] ({tx:.2f},{ty:.2f})')
        self._advance_waypoint()
        # 여기서 바로 새 목표를 향한 명령을 계산/발행한다 — 안 그러면 이번 tick은
        # (이미 지나친) 옛 목표를 향하던 명령 그대로 다음 tick(최대 1.2초)까지
        # "관성 주행"하게 된다.
        self._drive_toward_target()

    def _on_timer(self):
        if self._pose is None:
            return

        if self._dwell_until is not None:
            if time.monotonic() < self._dwell_until:
                self._pub.publish(Twist())  # 정지 유지 — 수신호가 twist_mux에서 이 위로 지나갈 수 있게
                return
            self._dwell_until = None
            self._advance_waypoint()
            self._drive_toward_target()  # 코너 통과와 동일한 이유로 바로 발행
            return

        x, y = self._pose.position.x, self._pose.position.y
        tx, ty = WAYPOINTS[self._index]
        distance = math.hypot(tx - x, ty - y)

        if distance < ARRIVAL_TOLERANCE:
            self._on_arrival()
            return

        self._drive_toward_target()

    def _drive_toward_target(self):
        """현재 self._index 목표를 향한 조향/속도를 계산해 즉시 발행한다."""
        x, y = self._pose.position.x, self._pose.position.y
        yaw = yaw_from_quaternion(self._pose.orientation)
        tx, ty = WAYPOINTS[self._index]
        distance = math.hypot(tx - x, ty - y)

        prev_index = self._index - self._dir
        if 0 <= prev_index < len(WAYPOINTS):
            prev_point = WAYPOINTS[prev_index]
        else:
            prev_point = (tx, ty)
        lookahead_dist = max(MIN_LOOKAHEAD_DISTANCE,
                             self._last_linear_speed * CONTROL_PERIOD_SEC * LOOKAHEAD_TIME_FACTOR)
        aim_x, aim_y = lookahead_point(prev_point, (tx, ty), (x, y), lookahead_dist)
        heading_error = normalize_angle(math.atan2(aim_y - y, aim_x - x) - yaw)

        # 제어 타이머는 use_sim_time=True라 시뮬레이션 시계 기준으로 틱이 돈다.
        # 이 월드는 소품이 많아 실시간 배속(RTF)이 크게 흔들리는데(navi_factory.sdf
        # 물리 설정 주석 참고), dt를 벽시계(time.monotonic())로 재면 틱 사이 sim-초는
        # 늘 CONTROL_PERIOD_SEC 그대로인데 벽시계 dt만 배속에 따라 요동쳐서 미분
        # 게인이 매 틱 제멋대로 커지거나 작아진다 — sim 시계로 재야 CONTROL_PERIOD_SEC를
        # 늘려 얻은 "느긋한 페이싱"의 의도가 실제로 지켜진다.
        now = self.get_clock().now()
        derivative = 0.0
        if self._prev_heading_error is not None and self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds / 1e9
            if dt > 1e-3:
                derivative = normalize_angle(heading_error - self._prev_heading_error) / dt
        self._prev_heading_error = heading_error
        self._prev_time = now

        cmd = Twist()
        pd = ANGULAR_GAIN * heading_error + ANGULAR_DAMPING * derivative
        target_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, pd))
        cmd.linear.x = (LINEAR_SPEED * speed_factor(abs(heading_error))
                        * distance_speed_factor(distance))
        if cmd.linear.x > 0.0:
            # 실제로 굴러가는(전진) 동안에만 각속도 변화를 slew-rate로 눌러 휘청거림을
            # 줄인다. heading_error가 FULL_STOP_ANGLE을 넘어 제자리 회전만 하는
            # 상태(linear.x == 0)에서까지 이 제한을 걸었더니, 정지 마찰을 이길 만큼의
            # 각속도까지 서서히 램프업하는 동안 로봇이 사실상 못 돌고 그 자리에 멈춰
            # 서버리는 걸 실측(diag pos 고정)으로 확인했다 — 정지 회전은 즉시 최대
            # 각속도로 반응해야 한다.
            angular_delta = max(-MAX_ANGULAR_STEP,
                                min(MAX_ANGULAR_STEP, target_angular - self._last_angular))
            cmd.angular.z = self._last_angular + angular_delta
        else:
            cmd.angular.z = target_angular
        self._pub.publish(cmd)
        self._last_linear_speed = cmd.linear.x
        self._last_angular = cmd.angular.z

        self._tick_count += 1
        if self._tick_count % 5 == 0:  # 대략 1.5초(실제 시간)마다 진단 로그
            self.get_logger().info(
                f'diag pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}deg '
                f'target=({tx:.2f},{ty:.2f}) aim=({aim_x:.2f},{aim_y:.2f}) '
                f'lookahead={lookahead_dist:.2f} dist={distance:.2f} '
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
