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

# 2026-07-28-JaeSeong
# package install 은 setup_infer_env.py 를 실행
# ===================================================================================
from collections import deque, namedtuple
from pathlib import Path
import time

import numpy as np
from robot_control.vision_hand.capture.extractor import FeatureExtractor, HAND_DIM
from robot_control.vision_hand.inference.predict import load_model as load_signal_model
from robot_control.vision_hand.inference.predict import SignalStabilizer
from robot_control.vision_hand.inference.ui import Hud
import torch


# 추론 스레드 → 렌더 스레드로 넘기는 한 프레임분 결과.
#
# 왜 튜플로 묶어서 넘기는가:
#     렌더 스레드가 self._stabilizer 를 직접 읽으면 레이스가 난다. update() 는
#     probs/top/confirmed/confirmed_idx/status/emergency 를 락 없이 순서대로 고치는데,
#     그 중간에 읽으면 probs 는 이번 프레임 것이고 confirmed_idx 는 지난 프레임 것인
#     뒤섞인 화면이 나온다. (GIL 덕에 배열이 깨지지는 않지만 조합이 깨진다)
#     그래서 update() 가 **끝난 뒤** 한 번에 스냅샷을 떠서 그 참조만 넘긴다.
#
#     namedtuple 이라 만들어진 뒤에는 아무도 못 고친다. 다음 프레임은 새 튜플을
#     만들므로, 렌더가 이전 튜플을 들고 있어도 참조 카운트 덕에 그대로 살아 있다.
#     (C++ 의 prev/cur 더블 버퍼링에 해당하는 일이 파이썬에서는 저절로 일어난다)
#
# frame 도 여기 들어간다: Hud.render() 는 프레임 위에 패널을 in-place 로 그린다.
# 스냅샷 이후 추론 쪽은 이 프레임을 다시 건드리지 않으므로 소유권이 렌더로 넘어간 셈이다.
RenderPayload = namedtuple(
    'RenderPayload',
    'frame labels probs top confirmed_idx threshold confirmed status emergency')

# 가중치는 패키지 루트의 models/ 에 있다(launch/, urdf/, config/ 와 형제).
#   .../src/robot_control/robot_control/camera_node/inference.py
#   parents[0]=camera_node  parents[1]=robot_control(파이썬 패키지)  parents[2]=robot_control(패키지 루트)
# parents[1] 이면 존재하지 않는 robot_control/robot_control/models 를 가리켜 FileNotFoundError 가 났다.
# 주의: --symlink-install 전제다. resolve() 가 심링크를 따라 src/ 로 되돌아가야 이 경로가 맞는다.
#       심링크 없는 일반 빌드로 바꾸면 setup.py data_files 에 models/ 를 넣고
#       get_package_share_directory('robot_control') 기준으로 다시 잡아야 한다.
MODEL_PATH = Path(__file__).resolve().parents[2] / 'models' / 'sign_classifier.pt'

# signal-vision 라벨 -> LABELS 매핑. 없는 키(back/slow/idle)는 .get()이 알아서 None.
LABEL_MAP = {
    'stop': 'STOP',
    'come': 'FORWARD',
    'left_go': 'LEFT',
    'right_go': 'RIGHT',
}
# ===================================================================================


class InferenceBase:
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

    def take_render_payload(self):
        """
        직전 infer() 가 남긴 렌더용 결과를 꺼낸다(꺼내면 비운다). 워커 스레드에서 호출.

        반환:
            show() 에 그대로 넘길 객체, 또는 None(그릴 것 없음).

        기본 구현은 None — GUI 를 안 쓰는 구현체는 이 메서드를 건드릴 필요가 없다.
        """
        return None

    def show(self, payload):
        """
        take_render_payload() 가 준 결과를 화면에 그린다. **렌더 스레드**에서 호출된다.

        인자:
            payload: take_render_payload() 의 반환값. None 이 올 수 있다.

        반환:
            True  — 사용자가 창에서 종료를 요청했다(q/ESC). 호출부가 노드를 내린다.
            False/None — 계속 진행.

        스레드 계약: 이 메서드만 렌더 스레드에서 불린다. load_model()/infer() 이
            쓰는 객체(MediaPipe 추출기, 모델, 안정화 필터)를 **절대 건드리면 안 된다** —
            그쪽은 추론 워커 1개 전용이라는 위 계약 아래 락 없이 돌고 있다.
            그릴 데이터는 전부 payload 안에 들어 있어야 한다.

        기본 구현은 아무것도 하지 않는다(GUI 없음).
        """

    def close(self):
        """GUI 창 등 리소스를 정리한다. 렌더 스레드가 끝난 뒤 1회 호출된다."""


class GestureInference(InferenceBase):

    def __init__(self, logger):
        super().__init__(logger)
        # 실체는 load_model() 에서 채운다. 여기서 미리 None 으로 두는 이유:
        # load_model() 이 실패해도 조립자가 close() 를 부르기 때문이다.
        self._hud = None
        self._pending = None

    def load_model(self):
        """
        모델/파이프라인을 로딩해 self 에 보관한다. 워커 시작 전 1회 호출된다.

        예)
            import mediapipe as mp
            self._hands = mp.solutions.hands.Hands(max_num_hands=1)

        무거운 초기화(가중치 로딩 등)는 반드시 여기서 한 번만 한다.
        infer() 안에서 매 프레임 하면 처리율이 무너진다.

        여기서 만든 객체는 **용도별로 한 스레드에만 묶인다.** 워커는 추론/렌더 둘이지만
        둘이 같은 객체를 건드리지는 않는다:
            추론 워커 전용 — _model, _extractor, _window, _stabilizer, _device, _t0
            렌더 워커 전용 — _hud
            양쪽이 읽음   — _labels (만든 뒤 바뀌지 않으므로 안전)
        그래서 MediaPipe Hands 처럼 스레드 안전하지 않은 객체도 그대로 써도 된다.
        새 객체를 추가할 때는 어느 쪽 전용인지 정해서 이 목록에 적어 둔다.
        """
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._model, self._labels, self._num_frames = load_signal_model(MODEL_PATH, self._device)
        self._extractor = FeatureExtractor()
        self._window = deque(maxlen=self._num_frames)
        self._stabilizer = SignalStabilizer(
            threshold=0.8, consecutive=5, ema=0.4, release_grace=1.0)
        self._t0 = time.monotonic()
        # HUD 는 폰트 로더만 들고 있다(창은 첫 render 때 열린다). 창을 여는 것도 닫는 것도
        # 렌더 스레드 쪽이라, cv2 HighGUI 호출이 한 스레드에 모인다.
        self._hud = Hud()
        self._pending = None

    def infer(self, frame):
        """
        프레임 한 장을 보고 수신호 라벨을 반환한다. 워커 스레드에서 호출된다.

        인자:
            frame: numpy.ndarray, shape=(height, width, 3), dtype=uint8, 채널 순서 BGR.
                   (OpenCV 기본 순서. RGB 가 필요하면 cv2.cvtColor 로 직접 변환)

        반환:
            LABELS 중 하나의 문자열 → 그대로 gesture 토픽으로 발행된다.
            None → 이번 프레임은 판단 불가. 아무것도 발행하지 않는다.

        주의:
            - 여기서 오래 걸려도 캡처는 멈추지 않는다. 대신 그동안 들어온 프레임은
              버려지고 가장 최신 것만 남는다.
            - 예외는 호출부가 잡아 로그만 남기므로 워커는 죽지 않는다.
            - LABELS 에 없는 문자열을 반환하면 발행되지 않고 경고만 남는다.
        """
        try:
            return self._infer_impl(frame)
        finally:
            # try/finally 인 이유: 아래 구현은 "버퍼 미충족", "사람 미감지", "미확정",
            # "라벨 확정" 네 갈래로 빠져나가는데 어느 쪽이든 화면은 갱신돼야 한다.
            # 예외로 빠져나갈 때도 마지막 상태는 남겨 둔다(그래야 화면이 안 멈춘다).
            self._pending = self._snapshot(frame)

    def _snapshot(self, frame):
        """이번 프레임의 결과를 렌더 스레드에 넘길 불변 묶음으로 뜬다. 추론 스레드 전용."""
        s = self._stabilizer
        # 이 시점은 stabilizer.update() 가 이미 끝난 뒤다. 그래서 아래 6개 값이
        # 전부 같은 프레임 것임이 보장된다 — 이게 스냅샷을 여기서 뜨는 이유다.
        return RenderPayload(
            frame=frame, labels=self._labels, probs=s.probs, top=s.top,
            confirmed_idx=s.confirmed_idx, threshold=s.threshold,
            confirmed=s.confirmed, status=s.status, emergency=s.emergency)

    def take_render_payload(self):
        """직전 infer() 의 스냅샷을 꺼내고 비운다. 추론 스레드에서만 부른다."""
        # 락이 없어도 되는 이유: 이 메서드도 _pending 을 쓰는 infer() 도 추론 워커
        # 한 스레드에서만 돈다. 렌더 스레드는 _pending 을 아예 안 본다 — 조립자인
        # node.py 가 이 반환값을 받아 자기 Condition 아래에서 렌더에 건네준다.

        # 더블 버퍼 스왑
        payload, self._pending = self._pending, None
        return payload

    def _infer_impl(self, frame):
        """infer() 의 실제 본문. 계약과 주의사항은 infer() 독스트링 참고."""
        timestamp_ms = int((time.monotonic() - self._t0) * 1000)
        hand_result, pose_result = self._extractor.detect(frame, timestamp_ms)
        draw_detections(frame, hand_result, pose_result)   # 렌더용 — 손/포즈 랜드마크를 프레임에 직접 그린다
        self._window.append(FeatureExtractor.vector(hand_result, pose_result))

        if len(self._window) < self._num_frames:
            return None

        arr = np.stack(self._window)
        pose_ratio = float((arr[:, HAND_DIM * 2:] != 0).any(axis=1).mean())
        if pose_ratio < 0.3:
            self._stabilizer.mark_person_absent(is_blurred=False)
            return None

        x = torch.from_numpy(arr[None]).to(self._device)
        with torch.no_grad():
            raw_probs = torch.softmax(self._model(x), dim=1)[0].cpu().numpy()
        self._stabilizer.update(raw_probs, self._labels, is_blurred=False)

        idx = self._stabilizer.confirmed_idx
        if idx is None:
            return None

        return LABEL_MAP.get(self._labels[idx])

    def show(self, payload):
        """
        추론 결과를 HUD 창에 그린다. 렌더 스레드 전용.

        Signal-Vision 의 show_gui() 를 쓰지 않는 이유: 그쪽은 predict.py 의 CLI 루프를
        대신 돌려주는 함수라, 호출하면 모듈 전역에 자기만의 _Runtime 을 만들면서
        모델을 다시 로딩하고 카메라를 다시 연다(우리 FrameSource 와 장치가 겹친다).
        게다가 그 _Runtime 은 우리가 추론한 결과를 모르므로 그릴 것도 없다.
        그래서 한 단계 아래인 Hud.render() 를 직접 부른다.
        """
        if payload is None:
            return False

        # 허재성 여기 렌더코드 떔빵인데, 기존 show_gui 함수 수정해서 해주면 됨.
        # 확정된 신호가 있으면 초록, 없으면 붉은 기 — 헤더 색으로 한눈에 구분한다.
        header_color = (0, 220, 0) if payload.confirmed_idx is not None else (80, 80, 255)
        return self._hud.render(
            payload.frame, payload.labels, payload.probs, payload.top,
            payload.confirmed_idx, payload.threshold,
            header=f'확정: {payload.confirmed}', header_color=header_color,
            status=payload.status, emergency=payload.emergency)

    def close(self):
        """HUD 창을 닫는다. 렌더 스레드가 끝난 뒤 조립자가 부른다."""
        if self._hud is not None:       # load_model() 전에 종료된 경우
            self._hud.close()
