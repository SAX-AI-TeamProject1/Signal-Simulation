# Last updated: 2026-07-15
"""Signal-Vision의 /upper_body_pose(JSON)를 구독해 tracker_robot의 18개 관절 위치 명령으로
변환·발행하는 노드.

Signal-Vision 쪽 계약(doc/design.md, src/inference/bridge.py 참고):
    /upper_body_pose, std_msgs/String, data = JSON:
    {"joints": {"left_shoulder": [x,y,z], ...} (8개, 항상 있음),
     "left_hand": {"wrist": [x,y,z], "thumb_cmc": [...], ...} (21개, 검출 안 되면 키 자체가 없음),
     "right_hand": {...} (위와 동일),
     "timestamp": ...}
좌표는 MediaPipe 정규화 값 그대로(가공 없음) — 여기서 로봇 관절각으로 바꾸는(리타겟팅) 계산을
전부 담당한다.

주의: 이 리타겟팅은 정식 역기구학(IK)이 아니라 벡터 각도 기반의 단순 근사다 — "사람 팔의
대략적인 자세를 로봇이 시각적으로 따라 하는" 데모 목적이며, 정밀한 동작 재현을 보장하지 않는다.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# controllers.yaml의 arm_position_controller.joints 순서와 반드시 동일해야 한다
# (Float64MultiArray는 이름이 아니라 순서로 관절을 구분하므로 순서가 어긋나면 엉뚱한 관절이 움직인다).
JOINT_ORDER = [
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow", "left_wrist",
    "left_thumb_joint", "left_index_joint", "left_middle_joint", "left_ring_joint", "left_pinky_joint",
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow", "right_wrist",
    "right_thumb_joint", "right_index_joint", "right_middle_joint", "right_ring_joint", "right_pinky_joint",
]

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
# 손가락별 (mcp, pip, tip) 랜드마크 이름 — hand_joint_positions()가 보내는 21개 이름 중 일부
FINGER_LANDMARKS = {
    "thumb": ("thumb_cmc", "thumb_mcp", "thumb_tip"),
    "index": ("index_mcp", "index_pip", "index_tip"),
    "middle": ("middle_mcp", "middle_pip", "middle_tip"),
    "ring": ("ring_mcp", "ring_pip", "ring_tip"),
    "pinky": ("pinky_mcp", "pinky_pip", "pinky_tip"),
}


def _v(p) -> np.ndarray:
    return np.array(p, dtype=np.float64)


def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """두 벡터 사이 각도(라디안). 둘 중 하나라도 길이가 0에 가까우면 0 반환(정의 불가 방지)."""
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_theta = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(math.acos(cos_theta))


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def arm_angles(joints: dict, side: str) -> tuple[float, float, float, float]:
    """어깨(shoulder, elbow 등) 8관절 좌표에서 (shoulder_pitch, shoulder_roll, elbow, wrist) 근사.

    MediaPipe 좌표계: x/y는 화면 기준 정규화(0~1, y는 아래로 증가), z는 카메라 기준 상대 깊이.
    torso(어깨-어깨) 벡터를 기준축으로 삼아 어깨→팔꿈치, 팔꿈치→손목 벡터의 상대 각도를 구한다.
    """
    ls, rs = _v(joints["left_shoulder"]), _v(joints["right_shoulder"])
    shoulder, elbow, wrist = (
        _v(joints[f"{side}_shoulder"]),
        _v(joints[f"{side}_elbow"]),
        _v(joints[f"{side}_wrist"]),
    )
    upper_arm = elbow - shoulder  # 어깨 → 팔꿈치
    forearm = wrist - elbow       # 팔꿈치 → 손목
    torso_x = (rs - ls) if side == "left" else (ls - rs)  # 몸통 좌우축 (팔이 뻗는 기준 옆방향)

    # shoulder_pitch: 팔을 앞뒤로 드는 정도 — 위팔의 수직(y) 성분으로 근사
    # (MediaPipe y는 아래로 증가하므로 위로 들수록 upper_arm.y가 작아짐/음수)
    shoulder_pitch = _clip(-upper_arm[1] * math.pi, -1.9, 1.9)
    # shoulder_roll: 팔을 옆으로 벌리는 정도 — 위팔이 몸통 좌우축과 이루는 각도
    shoulder_roll = _clip(_angle_between(upper_arm, torso_x), -0.2, 1.8)
    # elbow: 위팔-아래팔 사이 각도가 좁을수록(팔이 접힐수록) 관절각은 커짐(0=폄, 크면 굽힘)
    elbow = _clip(math.pi - _angle_between(upper_arm, forearm), 0.0, 2.4)
    # wrist: 별도 손목 회전 랜드마크가 없어 근사가 마땅치 않으므로 중립(0)으로 고정
    # (손 랜드마크가 있으면 아래 wrist_bend_from_hand로 보정)
    wrist = 0.0
    return shoulder_pitch, shoulder_roll, elbow, wrist


def wrist_bend_from_hand(joints: dict, hand: dict | None, side: str) -> float:
    """손 랜드마크가 있으면 (팔꿈치→손목) 대비 (손목→중지뿌리) 방향으로 손목 굽힘을 근사."""
    if hand is None:
        return 0.0
    elbow, wrist = _v(joints[f"{side}_elbow"]), _v(joints[f"{side}_wrist"])
    forearm = wrist - elbow
    palm = _v(hand["middle_mcp"]) - _v(hand["wrist"])
    # forearm 방향 대비 palm이 꺾인 각도를 -1.2~1.2 범위로 근사 (부호는 신경 안 씀 — 데모 목적)
    return _clip(_angle_between(forearm, palm) - math.pi / 2, -1.2, 1.2)


def finger_curl(hand: dict | None, finger: str) -> float:
    """손가락 하나의 굽힘 정도를 0(폄)~1.5(굽힘)로 근사.

    mcp→pip, pip→tip 두 마디 벡터 사이 각도가 클수록(방향이 꺾일수록) 더 굽은 것으로 본다.
    """
    if hand is None:
        return 0.0
    mcp_name, pip_name, tip_name = FINGER_LANDMARKS[finger]
    mcp, pip, tip = _v(hand[mcp_name]), _v(hand[pip_name]), _v(hand[tip_name])
    return _clip(_angle_between(pip - mcp, tip - pip), 0.0, 1.5)


class MotionRetargetNode(Node):
    """/upper_body_pose(JSON) → /arm_position_controller/commands(Float64MultiArray)."""

    def __init__(self) -> None:
        super().__init__("motion_retarget_node")
        self._last_angles = {name: 0.0 for name in JOINT_ORDER}  # 마지막으로 계산된 값 유지(누락분 보존용)
        self._sub = self.create_subscription(String, "/upper_body_pose", self._on_pose, 10)
        self._pub = self.create_publisher(Float64MultiArray, "/arm_position_controller/commands", 10)
        self.get_logger().info("motion_retarget_node 시작 — /upper_body_pose 구독 중")

    def _on_pose(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON 파싱 실패, 이번 프레임 건너뜀: {e}")
            return

        joints = payload.get("joints")
        if not joints:
            return  # Signal-Vision은 사람이 검출됐을 때만 joints를 채워 보내므로, 없으면 아무것도 안 함

        left_hand = payload.get("left_hand")
        right_hand = payload.get("right_hand")

        for side, hand in (("left", left_hand), ("right", right_hand)):
            pitch, roll, elbow, _ = arm_angles(joints, side)
            wrist = wrist_bend_from_hand(joints, hand, side)
            self._last_angles[f"{side}_shoulder_pitch"] = pitch
            self._last_angles[f"{side}_shoulder_roll"] = roll
            self._last_angles[f"{side}_elbow"] = elbow
            self._last_angles[f"{side}_wrist"] = wrist
            if hand is not None:  # 이 손이 이번 프레임에 검출된 경우에만 손가락 값 갱신
                for finger in FINGER_NAMES:
                    self._last_angles[f"{side}_{finger}_joint"] = finger_curl(hand, finger)

        out = Float64MultiArray()
        out.data = [self._last_angles[name] for name in JOINT_ORDER]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MotionRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
