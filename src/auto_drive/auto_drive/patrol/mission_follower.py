# 2026-08-02
# 수신호 "파견" 주행 노드. waypoint_follower(고정 트랙 무한 왕복)와 별개의 새 노드다.
#
# 왜 새 노드인가: LEFT/RIGHT 수신호를 회전 Twist(cmd_vel_gesture)로 쓰면 신호가
# 끊긴 0.5초 뒤 twist_mux 가 gesture 소스를 버리고 순찰(cmd_vel_auto)이 도로
# 잡아서, 좌로 돌린 로봇을 트랙 방향으로 되돌려 버렸다. 그래서 LEFT/RIGHT 는
# "그 방향 존으로 다녀와라"는 파견 명령으로 바꾼다 — 명령의 수명이 신호 지속시간이
# 아니라 임무 완료까지가 되므로 이 싸움 자체가 사라진다. waypoint_follower 는
# 옛 레이아웃 하드코딩이라 손대지 않고 남겨 둔다(제어 함수는 여기서 가져다 쓴다).
#
# 동작: 수신호석(signal_point)에서 대기 → signal_dispatch(std_msgs/String, 존 이름)
# 수신 → tracks.yaml 의 해당 경로를 주행 → 원판(station)에서 잠깐 정차 → 경로
# 끝(수신호석)으로 복귀 → 다시 대기. 주행 중에 온 파견 명령은 무시한다.
#
# 대기 중에는 cmd_vel_auto 에 아무것도 발행하지 않는다 — 로봇은 어차피 서 있고,
# 발행을 멈추면 twist_mux 가 timeout 으로 auto 소스를 버려서 수신호 STOP
# (cmd_vel_gesture, 우선순위 50)이 그대로 통과한다.
#
# 조향/속도 제어는 waypoint_follower 와 동일한 법칙(pure-pursuit 조준점 + PD +
# slew-rate 제한)을 그대로 import 한다. 거기서 실측으로 잡은 스톨/헌팅/휘청거림
# 대책을 여기서 다시 발명하지 않기 위해서다 — 상수 튜닝도 그쪽 한 곳에서만 한다.

import math
from pathlib import Path
import time

from auto_drive.patrol.waypoint_follower import (ANGULAR_DAMPING,
                                                 ANGULAR_GAIN,
                                                 ARRIVAL_TOLERANCE,
                                                 CONTROL_PERIOD_SEC as
                                                 WF_CONTROL_PERIOD_SEC,
                                                 distance_speed_factor,
                                                 LINEAR_SPEED,
                                                 lookahead_point,
                                                 LOOKAHEAD_TIME_FACTOR,
                                                 MAX_ANGULAR,
                                                 MAX_ANGULAR_STEP as
                                                 WF_MAX_ANGULAR_STEP,
                                                 MIN_LOOKAHEAD_DISTANCE,
                                                 normalize_angle,
                                                 speed_factor,
                                                 yaw_from_quaternion)
from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml

# 경로의 어느 점이 원판(station)인지는 stations 좌표와 거리 대조로 정한다 —
# waypoint_follower 의 classify_waypoints 와 같은 이유(경로점은 코너 완화용으로
# 세분화돼 있어 인덱스를 못 박을 수 없다), 같은 허용 오차를 쓴다.
POINT_MATCH_TOLERANCE = 0.5  # m
STATION_DWELL_SEC = 2.0      # 원판 도착 시 정차 시간

# 제어 주기. waypoint_follower 의 1.2s 는 예전 RTF 요동(navi_factory.sdf 물리 스텝
# 수정 전) 대책으로 실측해 잡은 값인데, 그 주기로는 순항(3.33m/s) 중 한 틱에 최대
# 4m 를 가서 도착 반경(ARRIVAL_TOLERANCE 0.6m)을 틱과 틱 사이에 건너뛰는 일이
# 실제로 났다 — zone_nw 경로 [6]번(22.9m 직선 끝)을 1.17m 지나쳐 유턴하며 멈춘
# 실측 사례(2026-08-02). RTF≈1 이 된 지금은 주기를 줄여 틱당 이동을 반경 안으로
# 넣는다: 감속 구간 최악 케이스(목표 앞 ~3m, ≈1.9m/s)에서 0.6s 면 틱당 1.1m 씩
# 줄어들어 반경을 건너뛰지 못한다. legacy 인 waypoint_follower 의 상수는 그대로 둔다.
#
# 2026-08-02 추가: 위 0.6s 로도 순항(3.33m/s) 중에는 틱당 2.0m 를 간다 — 조향을
# 한 번 갱신할 때마다 2m 를 눈 감고 가는 셈이고, 도착 반경(0.6m)도 순항 중에는
# 여전히 건너뛸 수 있다(위 계산은 감속 구간만 따졌다). 커브 직후 긴 직선에서
# 좌우로 왔다갔다하는 증상이 여기서 온다고 보고 0.1s(10Hz)로 줄인다: 틱당 이동이
# 0.33m 가 되어 조준 거리(순항 3.0m)의 1/9 이라, 다음 갱신 전에 조준점을 지나쳐
# 버리는 일이 없어진다. 아래 MAX_ANGULAR_STEP 이 주기에 비례해 함께 줄어들므로
# 각가속 상한(rad/s^2)은 예전과 동일하게 유지된다.
CONTROL_PERIOD_SEC = 0.1
# 각속도 slew 제한은 "틱당" 상한이라, 주기만 줄이면 초당 허용 변화율(rad/s^2)이
# 그만큼 커져 휘청거림 대책이 약해진다. 원본과 같은 각가속 상한이 유지되도록
# 주기 비율로 줄인다 (1.2s 에서 0.5 → 0.6s 에서 0.25).
MAX_ANGULAR_STEP = WF_MAX_ANGULAR_STEP * (CONTROL_PERIOD_SEC / WF_CONTROL_PERIOD_SEC)


def load_dispatch_routes(tracks_yaml_path):
    """
    tracks.yaml 에서 signal_point / stations / routes 를 읽어 검증해 돌려준다.

    반환: (signal_point (x, y), stations {이름: (x, y)}, routes {이름: [(x, y), ...]})

    빠진 키·빈 경로·시작/끝이 signal_point 가 아닌 경로는 RuntimeError —
    파견 노드를 켜 놓고 조용히 아무 데도 못 가는 상태보다 시작 시점에 원인을
    분명히 내는 쪽이 낫다(signal_zone.load_zone_points 와 같은 패턴).
    """
    if not Path(tracks_yaml_path).is_file():
        raise RuntimeError(
            f'tracks_yaml_path 파일을 찾을 수 없습니다: {tracks_yaml_path!r}')
    with open(tracks_yaml_path) as f:
        data = yaml.safe_load(f) or {}

    signal_point = data.get('signal_point')
    stations = data.get('stations') or {}
    routes_raw = data.get('routes') or {}
    if signal_point is None or not routes_raw:
        raise RuntimeError(
            f'{tracks_yaml_path!r} 에 signal_point/routes 가 없습니다. '
            '파견 주행은 두 항목이 모두 있어야 동작합니다.')

    signal_point = tuple(signal_point[:2])
    stations = {name: tuple(point[:2]) for name, point in stations.items()}

    routes = {}
    for name, route in routes_raw.items():
        waypoints = [tuple(point[:2]) for point in (route.get('waypoints') or [])]
        if len(waypoints) < 2:
            raise RuntimeError(f'경로 {name!r} 의 waypoints 가 2개 미만입니다.')
        for end in (waypoints[0], waypoints[-1]):
            if math.hypot(end[0] - signal_point[0],
                          end[1] - signal_point[1]) > POINT_MATCH_TOLERANCE:
                raise RuntimeError(
                    f'경로 {name!r} 는 signal_point {signal_point} 에서 시작해 '
                    f'거기로 끝나야 합니다 (실제 양 끝: {waypoints[0]}, {waypoints[-1]}). '
                    '파견은 항상 수신호석 왕복이다 — 다른 시작/끝이 필요해지면 '
                    '그때 이 검증을 풀 것.')
        routes[name] = waypoints
    return signal_point, stations, routes


def station_indices(waypoints, stations, tolerance=POINT_MATCH_TOLERANCE):
    """경로점 중 어느 인덱스가 station(원판) 위인지 집합으로 돌려준다."""
    found = set()
    for i, (x, y) in enumerate(waypoints):
        if any(math.hypot(x - sx, y - sy) <= tolerance
               for sx, sy in stations.values()):
            found.add(i)
    return found


class MissionFollower(Node):
    """수신호석 대기 → 파견 명령 수신 → 존 왕복 → 복귀 대기를 반복한다."""

    def __init__(self):
        """tracks.yaml 의 경로를 읽고, 대기(WAITING) 상태로 시작한다."""
        super().__init__('mission_follower')

        self.declare_parameter('tracks_yaml_path', '')
        self.declare_parameter('station_dwell_sec', STATION_DWELL_SEC)

        tracks_yaml_path = self.get_parameter('tracks_yaml_path').value
        if not tracks_yaml_path:
            # waypoint_follower 는 경로 없이도 하드코딩 트랙으로 돌 수 있지만,
            # 이 노드는 경로가 곧 임무라서 없으면 존재 이유가 없다 → 즉시 실패.
            raise RuntimeError(
                'tracks_yaml_path 파라미터가 비어 있습니다. 파견 경로(routes)가 '
                '있는 tracks.yaml 경로를 넘겨야 합니다 '
                '(worlds/navi_factory/world/navi_factory/tracks.yaml).')
        self._signal_point, stations, self._routes = (
            load_dispatch_routes(tracks_yaml_path))
        self._station_dwell_sec = self.get_parameter('station_dwell_sec').value

        self._station_indices = {
            name: station_indices(waypoints, stations)
            for name, waypoints in self._routes.items()}
        for name, waypoints in self._routes.items():
            self.get_logger().info(
                f'파견 경로 {name}: 경로점 {len(waypoints)}개, '
                f'원판 인덱스 {sorted(self._station_indices[name])}')

        # 임무 상태. _route 가 None 이면 수신호석 대기(WAITING)다 — 상태 enum 을
        # 따로 두지 않는 이유는 "주행 중인가"가 곧 "_route 가 있는가"이기 때문.
        self._route = None          # 주행 중인 경로 이름
        self._index = 0             # 현재 목표 경로점
        self._dwell_until = None    # 원판 정차 중이면 그 해제 시각
        self._pose = None
        self._prev_heading_error = None
        self._prev_time = None
        self._last_linear_speed = 0.0
        self._last_angular = 0.0
        self._tick_count = 0

        self.create_subscription(Pose, 'pose_gt', self._on_pose, 10)
        self.create_subscription(String, 'signal_dispatch', self._on_dispatch, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)

        # 지금 무엇을 하고 있는지. 주행에는 안 쓰이고 기록(stop_logger)용이다.
        #
        # 이게 필요한 이유: 대기 중에는 cmd_vel_auto 를 아예 발행하지 않으므로,
        # 밖에서 보면 "수신호석에서 신호를 기다리는 중"과 "아무도 안 띄웠다"가
        # 똑같이 침묵으로 보인다. 정차(dwelling)도 0 을 쏘는 것뿐이라 estop 이나
        # 수신호로 선 것과 구분되지 않는다. 사유를 아는 건 이 노드뿐이다.
        #
        # 래치(transient_local)인 이유: 전환이 드물어서 기록 노드가 나중에 떠도
        # 마지막 상태를 받아야 한다. 매 틱 쏘는 estop 쪽과 다른 점이다.
        self._state_pub = self.create_publisher(
            String, 'mission_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._state = None
        self._set_state('waiting')

        self.create_timer(CONTROL_PERIOD_SEC, self._on_timer)
        self.get_logger().info('수신호석 대기 중 — signal_dispatch 를 기다립니다.')

    def _set_state(self, state):
        """상태가 바뀔 때만 발행한다. 값은 waiting / driving / dwelling."""
        if state == self._state:
            return
        self._state = state
        self._state_pub.publish(String(data=state))

    # ------------------------------------------------------------- 콜백

    def _on_pose(self, msg):
        self._pose = msg

    def _on_dispatch(self, msg):
        """
        파견 명령(존 이름) 수신. 대기 중일 때만 받는다.

        camera_node 는 라벨을 인식하는 동안 매 프레임(~12회/초) 다시 보내므로,
        주행 중 수신은 정상 상황이다 — 스팸 로그가 되지 않게 throttle 로 남긴다.
        """
        zone = msg.data
        if self._route is not None:
            self.get_logger().info(
                f'주행 중이라 파견 명령을 무시합니다: {zone!r} (현재 임무: {self._route})',
                throttle_duration_sec=5.0)
            return
        if zone not in self._routes:
            self.get_logger().warn(
                f'모르는 존이라 무시합니다: {zone!r} (알고 있는 경로: '
                f'{sorted(self._routes)})', throttle_duration_sec=5.0)
            return
        self._route = zone
        self._set_state('driving')
        # 경로점 0 은 수신호석(지금 서 있는 곳)이므로 목표는 1 부터다.
        self._index = 1
        self._prev_heading_error = None
        self._prev_time = None
        self.get_logger().info(f'파견 시작: {zone} — 경로점 {len(self._routes[zone])}개')
        if self._pose is not None:
            self._drive_toward_target()

    # ------------------------------------------------------------- 주행

    def _on_timer(self):
        if self._route is None or self._pose is None:
            return      # 대기 중 — 발행하지 않아야 gesture 가 mux 를 그대로 통과한다.

        if self._dwell_until is not None:
            if time.monotonic() < self._dwell_until:
                self._pub.publish(Twist())      # 정차 유지
                return
            self._dwell_until = None
            self._advance()
            if self._route is not None:
                self._set_state('driving')
                self._drive_toward_target()
            return

        x, y = self._pose.position.x, self._pose.position.y
        tx, ty = self._routes[self._route][self._index]
        if math.hypot(tx - x, ty - y) < ARRIVAL_TOLERANCE:
            self._on_arrival()
            return
        self._drive_toward_target()

    def _on_arrival(self):
        """경로점 도착. 원판이면 정차, 마지막 점이면 임무 종료, 그 외엔 다음 점."""
        waypoints = self._routes[self._route]
        tx, ty = waypoints[self._index]

        if self._index == len(waypoints) - 1:
            self.get_logger().info(
                f'임무 완료: {self._route} — 수신호석({tx:.2f},{ty:.2f}) 복귀, 다시 대기')
            # 마지막으로 0 을 한 번 쏘고 발행을 멈춘다. 이후에는 mux 의 auto
            # timeout(0.5초)이 지나면 gesture 가 다시 최우선 실사용 소스가 된다.
            self._pub.publish(Twist())
            self._last_linear_speed = 0.0
            self._last_angular = 0.0
            self._route = None
            self._set_state('waiting')
            return

        if self._index in self._station_indices[self._route]:
            self.get_logger().info(
                f'원판 도착: ({tx:.2f},{ty:.2f}) — {self._station_dwell_sec:.1f}초 정차')
            self._pub.publish(Twist())
            self._last_linear_speed = 0.0
            self._last_angular = 0.0
            self._dwell_until = time.monotonic() + self._station_dwell_sec
            self._set_state('dwelling')
            return

        self._advance()
        # 코너 통과 즉시 새 목표 명령을 발행한다 — 안 그러면 다음 tick(최대 1.2초)
        # 까지 옛 목표를 향한 명령으로 관성 주행한다(waypoint_follower 와 동일).
        self._drive_toward_target()

    def _advance(self):
        """다음 경로점으로 넘어간다. 파견 경로는 단방향이라 왕복 반전이 없다."""
        self._index += 1
        # 목표가 바뀌면 heading_error 가 코너 각도만큼 점프한다 — 미분항이 그걸
        # 변화량으로 잡는 derivative kick 을 막으려고 리셋한다(원본과 동일).
        self._prev_heading_error = None
        self._prev_time = None

    def _drive_toward_target(self):
        """
        현재 목표 경로점을 향한 조향/속도를 계산해 즉시 발행한다.

        수식과 상수는 전부 waypoint_follower 의 것이다 — 설명도 그쪽 주석을 볼 것.
        """
        waypoints = self._routes[self._route]
        x, y = self._pose.position.x, self._pose.position.y
        yaw = yaw_from_quaternion(self._pose.orientation)
        tx, ty = waypoints[self._index]
        distance = math.hypot(tx - x, ty - y)

        prev_point = waypoints[self._index - 1] if self._index > 0 else (tx, ty)
        lookahead_dist = max(
            MIN_LOOKAHEAD_DISTANCE,
            self._last_linear_speed * CONTROL_PERIOD_SEC * LOOKAHEAD_TIME_FACTOR)
        aim_x, aim_y = lookahead_point(prev_point, (tx, ty), (x, y), lookahead_dist)
        heading_error = normalize_angle(math.atan2(aim_y - y, aim_x - x) - yaw)

        now = self.get_clock().now()
        derivative = 0.0
        if self._prev_heading_error is not None and self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds / 1e9
            if dt > 1e-3:
                derivative = normalize_angle(
                    heading_error - self._prev_heading_error) / dt
        self._prev_heading_error = heading_error
        self._prev_time = now

        cmd = Twist()
        pd = ANGULAR_GAIN * heading_error + ANGULAR_DAMPING * derivative
        target_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, pd))
        cmd.linear.x = (LINEAR_SPEED * speed_factor(abs(heading_error))
                        * distance_speed_factor(distance))
        if cmd.linear.x > 0.0:
            angular_delta = max(-MAX_ANGULAR_STEP,
                                min(MAX_ANGULAR_STEP,
                                    target_angular - self._last_angular))
            cmd.angular.z = self._last_angular + angular_delta
        else:
            cmd.angular.z = target_angular
        self._pub.publish(cmd)
        self._last_linear_speed = cmd.linear.x
        self._last_angular = cmd.angular.z

        self._tick_count += 1
        if self._tick_count % 5 == 0:
            self.get_logger().info(
                f'diag [{self._route}:{self._index}] pos=({x:.2f},{y:.2f}) '
                f'yaw={math.degrees(yaw):.1f}deg target=({tx:.2f},{ty:.2f}) '
                f'dist={distance:.2f} heading_err={math.degrees(heading_error):.1f}deg '
                f'cmd=(lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f})')


def main():
    rclpy.init()
    node = MissionFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
