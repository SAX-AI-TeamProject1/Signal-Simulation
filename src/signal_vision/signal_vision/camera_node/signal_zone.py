"""
책임 5 — 수신호를 "받아도 되는 자리"인지 판정하는 게이트.

웹캠은 로봇 위치를 모르므로 수신호는 어디서든 인식된다. 그대로 두면 트랙을
달리는 중에 웹캠 앞에서 손을 흔드는 것만으로 순찰(우선순위 10)을 수신호(50)가
덮어버린다. 그래서 라벨→Twist 발행을 이 게이트로 막는다: 로봇이 신호 대기
지점(반경 안)에 있고, 신호수 스팟 쪽을 보고 있을 때만 명령이 나간다.

"사람이 실제로 있는가"는 여기서 확인하지 않는다 — 그건 웹캠 쪽에서 공짜로
검증된다(실제 손이 안 보이면 라벨 자체가 안 나온다). 여기가 맡는 것은
시뮬레이션 쪽 맥락 하나뿐이다: 맞는 자리에서 맞는 방향을 보고 있는가.
그래서 이 판정에는 센서가 필요 없고 pose_gt 하나면 된다.

STOP 도 존 밖에서는 안 나간다. 주행 중 안전은 estop_node(우선순위 100)의
몫이고, 수신호를 안전 장치로 겸용하면 "웹캠 앞에 누가 지나갔다"가 주행을
세우는 오작동이 된다.

좌표의 출처는 worlds/navi_factory/world/navi_factory/tracks.yaml
(waypoint_follower 와 같은 파일, 월드 SDF 옆에 있다)이다. 경로가
비어 있으면 게이트 자체가 만들어지지 않고 기존처럼 항상 발행한다 —
run_camera_node.sh 단독 실행(웹캠→수신호 단위 점검)이 pose_gt 없이도
돌아야 하기 때문이다.
"""

import math
from pathlib import Path
import time

import yaml


def load_zone_points(tracks_yaml_path):
    """
    tracks.yaml 에서 signal_point 와 hand_signal_spot 좌표를 읽는다.

    파일이나 키가 없으면 RuntimeError — 게이트를 켜 놓고 조용히 항상-닫힘이
    되는 것보다 시작 시점에 원인을 분명히 내는 쪽이 낫다(marker_vision 의
    가중치 검사, waypoint_follower 의 경로 검사와 같은 패턴).
    """
    if not Path(tracks_yaml_path).is_file():
        raise RuntimeError(
            f'tracks_yaml_path 가 지정됐지만 파일을 찾을 수 없습니다: '
            f'{tracks_yaml_path!r}')
    with open(tracks_yaml_path) as f:
        data = yaml.safe_load(f) or {}
    signal_point = data.get('signal_point')
    spot = data.get('hand_signal_spot')
    if signal_point is None or spot is None:
        raise RuntimeError(
            f'{tracks_yaml_path!r} 에 signal_point/hand_signal_spot 이 없습니다. '
            '수신호 게이트는 두 좌표가 모두 있어야 동작합니다.')
    return tuple(signal_point), tuple(spot)


def yaw_from_quaternion(qx, qy, qz, qw):
    """
    쿼터니언에서 yaw(rad)만 뽑는다.

    waypoint_follower.py 의 동일 함수를 복사한 것 — 수식 6줄 때문에
    signal_vision 이 auto_drive 패키지에 의존하게 만들지 않으려는 선택이다.
    """
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def normalize_angle(a):
    """각도를 [-pi, pi] 로 접는다. 출처는 yaw_from_quaternion 과 같다."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SignalZoneGate:
    """
    "신호 대기 지점에서 신호수 쪽을 보고 있는가"를 pose 만으로 판정한다.

    스레드 규칙: update_pose() 는 실행기 스레드(pose_gt 콜백)가, allows() 는
    캡처+추론 워커가 부른다. 공유 상태는 bool 과 float 각 하나뿐이고 대입은
    GIL 아래에서 원자적이라 락을 두지 않는다 — 한 틱 낡은 판정을 읽는 것이
    최악인데, pose_gt 는 물리 스텝마다 오므로 그 낡음은 ms 단위다.
    """

    def __init__(self, signal_point, spot, radius, half_angle_rad,
                 stale_sec=1.0):
        """
        게이트 파라미터를 검증하고 저장한다.

        인자:
            signal_point: 로봇이 서서 신호를 받는 지점 (x, y)
            spot: 신호수가 서는 지점 (x, y) — 방향 판정의 목표
            radius: signal_point 중심의 허용 반경 (m)
            half_angle_rad: 로봇 heading 과 스팟 방위각의 허용 오차 반각 (rad)
            stale_sec: pose 가 이보다 오래 안 오면 닫힘 — 게이트를 켜 놓고
                위치를 모르는 채 명령을 내보내는 것은 게이트가 없는 것과 같다
                (fail-closed)
        """
        if radius <= 0.0 or half_angle_rad <= 0.0:
            raise ValueError(
                f'반경과 반각은 양수여야 합니다 '
                f'(radius={radius}, half_angle_rad={half_angle_rad})')
        self._signal_point = signal_point
        self._spot = spot
        self._radius = radius
        self._half_angle = half_angle_rad
        self._stale_sec = stale_sec
        self._allowed = False
        self._updated_at = None

    def update_pose(self, x, y, qx, qy, qz, qw):
        """Pose 하나로 판정을 갱신하고 그 결과를 돌려준다(로그용)."""
        sx, sy = self._signal_point
        in_zone = math.hypot(x - sx, y - sy) <= self._radius

        # 방향은 "지금 서 있는 곳→스팟" 방위각과 로봇 heading 의 차로 잰다.
        # 존 판정과 독립이라 존 밖에서도 계산은 되지만, 둘 다 참일 때만 연다.
        tx, ty = self._spot
        yaw = yaw_from_quaternion(qx, qy, qz, qw)
        bearing = math.atan2(ty - y, tx - x)
        facing = abs(normalize_angle(bearing - yaw)) <= self._half_angle

        self._allowed = in_zone and facing
        self._updated_at = time.monotonic()
        return self._allowed

    def allows(self):
        """지금 명령을 내보내도 되는가. 워커 스레드에서 매 라벨마다 불린다."""
        if self._updated_at is None:
            return False    # pose 를 한 번도 못 봤다 — 위치를 모르면 닫는다.
        if time.monotonic() - self._updated_at > self._stale_sec:
            return False    # pose 가 끊겼다(시뮬 다운 등). 같은 이유로 닫는다.
        return self._allowed
