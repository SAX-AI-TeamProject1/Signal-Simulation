# 벤더 복사본 — Signal-transport-perception(src/infer.py)의 얇은 YOLO wrapper를 그대로
# 가져왔다. camera_node/vision_hand가 Signal-Vision을 벤더링하는 것과 동일한 패턴:
# 무거운 실제 학습/데이터 파이프라인은 그 별도 레포에 있고, 여기는 추론에 필요한
# 최소한만 복사해서 robot_control(ROS 노드)이 별도 venv 없이 바로 import할 수 있게 한다.
#
# ultralytics는 이 패키지를 실행하는 인터프리터(보통 시스템 python3, run_bringup.sh의
# torch/mediapipe 체크와 동일한 이유)에 설치돼 있어야 한다.

from ultralytics import YOLO


def load_model(weights: str) -> YOLO:
    return YOLO(weights)


def predict(model: YOLO, source, conf: float = 0.4):
    return model.predict(source=source, conf=conf, verbose=False)
