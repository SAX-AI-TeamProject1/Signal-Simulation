"""
책임 3 — 추론. 이 프로젝트의 인지(perception) 경계면.

doc/design.md: 카메라 기반 이미지 프로세싱은 별도 리포지토리에서 개발되며,
이 리포지토리는 처리된 인지 데이터를 "외부 입력"으로 취급해야 한다.
그래서 이 파일은 알고리즘을 담지 않고 인터페이스만 정의한다.

이 파일도 rclpy 를 import 하지 않는다. 로거는 주입받는다(덕 타이핑).
그래서 ROS 없이 가짜 로거만 넣어도 테스트가 돌아간다.

모델이 준비되면 이 파일에 구현 클래스를 추가하거나, 외부 패키지의 클래스를
node.py 의 _build_inference() 에서 만들어 주면 된다.
"""

import time


class GestureInference:
    """
    프레임 → 수신호 라벨. 실제 모델은 이 클래스를 상속해서 붙인다.

    ┌─ 붙이는 방법 ─────────────────────────────────────────────────────┐
    │  class MyHandGesture(GestureInference):                            │
    │      def load_model(self):                                         │
    │          import mediapipe as mp                                    │
    │          self._hands = mp.solutions.hands.Hands(max_num_hands=1)   │
    │                                                                    │
    │      def infer(self, frame):                                       │
    │          ...                                                       │
    │          return 'STOP'                                             │
    │                                                                    │
    │  그리고 node.py 의 CameraNode._build_inference() 가 이걸 만들게     │
    │  한 줄만 바꾼다.                                                    │
    └────────────────────────────────────────────────────────────────────┘

    스레드 계약: load_model() 과 infer() 는 **모두 워커 스레드 1개**에서만 불린다.
    (load_model 은 워커 시작 직전에 1회) 따라서 여기서 만든 객체는 한 스레드에서만
    쓰이므로, MediaPipe Hands 처럼 스레드 안전하지 않은 객체도 그대로 써도 된다.
    """

    def __init__(self, logger):
        """
        로거만 받아 둔다. 모델 로딩은 load_model() 에서 한다.

        인자:
            logger: .info/.warn/.error 를 가진 객체(rclpy 로거를 그대로 받는다).
                    rclpy 타입을 직접 import 하지 않으므로 테스트에서 가짜 객체를
                    넣어 ROS 없이 돌릴 수 있다.
        """
        self._logger = logger

    def load_model(self):
        """
        모델/파이프라인을 로딩해 self 에 보관한다. 워커 시작 전 1회 호출된다.

        무거운 초기화(가중치 로딩 등)는 반드시 여기서 한 번만 한다.
        infer() 안에서 매 프레임 하면 처리율이 무너진다.
        """
        raise NotImplementedError

    def infer(self, frame):
        """
        프레임 한 장을 보고 수신호 라벨을 반환한다. 워커 스레드에서 호출된다.

        인자:
            frame: numpy.ndarray, shape=(height, width, 3), dtype=uint8, 채널 순서 BGR.
                   (OpenCV 기본 순서. RGB 가 필요하면 cv2.cvtColor 로 직접 변환)

        반환:
            labels.LABELS 중 하나의 문자열 → gesture / cmd_vel_gesture 로 발행된다.
            None → 이번 프레임은 판단 불가. 아무것도 발행하지 않는다.

        주의:
            - 여기서 오래 걸려도 캡처는 멈추지 않는다. 대신 그동안 들어온 프레임은
              버려지고 가장 최신 것만 남는다.
            - 예외는 호출부가 잡아 로그만 남기므로 워커는 죽지 않는다.
            - LABELS 에 없는 문자열을 반환하면 발행되지 않고 경고만 남는다.
        """
        raise NotImplementedError


class StubGestureInference(GestureInference):
    """
    모델이 붙기 전까지 쓰는 자리표시자. 아무것도 인식하지 않는다.

    50ms 를 자는 이유: 실제 모델의 추론 시간을 흉내내서 파이프라인(큐 깊이, 유실률,
    fps 기본값 20)이 의도대로 도는지 미리 확인하기 위한 것이다.
    모델이 붙으면 이 클래스는 통째로 지운다.
    """

    FAKE_LATENCY_SEC = 0.05

    def load_model(self):
        """로딩할 모델이 없다. 스텁이라는 사실만 크게 알린다."""
        self._logger.warn(
            '추론이 StubGestureInference 로 돌고 있습니다 — 라벨이 절대 나가지 않습니다. '
            'GestureInference 를 상속한 실제 구현으로 교체하세요.')

    def infer(self, frame):
        """추론 시간만 흉내내고 항상 None(판단 불가)을 반환한다."""
        time.sleep(self.FAKE_LATENCY_SEC)
        return None
