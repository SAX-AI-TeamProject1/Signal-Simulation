# Last updated: 2026-07-27
"""pub_test/vision/의 vendor된 Signal-Vision 코드가 실제 웹캠으로 동작하는지 확인하는 단독 스크립트.
ROS 없이 실행 가능 (rclpy import 없음). 실행: .venv-infer/bin/python examples/src/pub_test/smoke_test_vision.py
종료: 표시되는 창에서 q 또는 ESC.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pub_test.vision.inference.function import close, infer, show_gui


def main() -> None:
    print("웹캠 열고 모델 로드 중... (첫 infer() 호출에서 초기화)")
    try:
        while True:
            signal, confidence = infer()
            print(f"\r신호: {signal:10s} 신뢰도: {confidence:.2f}", end="", flush=True)
            if show_gui():
                break
    finally:
        close()
        print("\n종료")


if __name__ == "__main__":
    main()
