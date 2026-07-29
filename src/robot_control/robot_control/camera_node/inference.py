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

# 2026-07-28-JaeSeong
# package install 은 setup_infer_env.py 를 실행
# ===================================================================================
from collections import deque
from pathlib import Path

import cv2   # show() 가 새 프레임 없이 이벤트만 펌핑할 때만 직접 쓴다 (아래 show() 참고)
import numpy as np
import torch

from robot_control.camera_node.swap_frame import LatestFrameBuffer
from robot_control.vision_hand.capture.extractor import HAND_DIM, FeatureExtractor, draw_detections   # + 신규
from robot_control.vision_hand.inference.predict import SignalStabilizer             # + 신규
from robot_control.vision_hand.inference.predict import load_model as load_signal_model  # 이름 충돌 회피용 alias
from robot_control.vision_hand.inference.ui import Hud   # 확률 패널·랜드마크 오버레이·창 표시

# 가중치는 패키지 루트의 models/ 에 있다(launch/, urdf/, config/ 와 형제).
#   .../src/robot_control/robot_control/camera_node/inference.py
#   parents[0]=camera_node  parents[1]=robot_control(파이썬 패키지)  parents[2]=robot_control(패키지 루트)
# parents[1] 이면 존재하지 않는 robot_control/robot_control/models 를 가리켜 FileNotFoundError 가 났다.
# 주의: --symlink-install 전제다. resolve() 가 심링크를 따라 src/ 로 되돌아가야 이 경로가 맞는다.
#       심링크 없는 일반 빌드로 바꾸면 setup.py data_files 에 models/ 를 넣고
#       get_package_share_directory('robot_control') 기준으로 다시 잡아야 한다.
MODEL_PATH = Path(__file__).resolve().parents[2] / "models" / "sign_classifier.pt"

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
    def show(self):
        """
        최근 추론 결과를 창에 그린다. **렌더 스레드에서만** 호출된다 — infer()/load_model()과는
        다른 스레드다. cv2.imshow/waitKey를 추론 스레드에서 부르면 그 GUI 호출 시간만큼
        추론이 지연되므로, node.py가 별도 렌더 스레드에서 이 메서드를 주기적으로 폴링한다.

        반환:
            True  — 사용자가 창 종료(q/ESC)를 요청했다. 렌더 루프만 멈추면 된다(노드는 계속 동작).
            False — 그릴 게 아직 없거나(추론 결과 대기 중) 계속 진행.
        """
        return False

    def close(self):
        """show() 가 연 창 등 리소스를 정리한다. destroy_node() 에서 호출된다."""
        pass


class GestureInference(InferenceBase):

    def __init__(self, logger):
        super().__init__(logger)
        #self._logger.

    """
    모델이 붙기 전까지 쓰는 자리표시자. 아무것도 인식하지 않는다.

    50ms 를 자는 이유: 실제 모델의 추론 시간을 흉내내서 파이프라인(큐 깊이, 유실률,
    fps 기본값 20)이 의도대로 도는지 미리 확인하기 위한 것이다.
    모델이 붙으면 이 클래스는 통째로 지운다.
    """
    def load_model(self):
        """
        모델/파이프라인을 로딩해 self 에 보관한다. 워커 시작 전 1회 호출된다.

        예)
            import mediapipe as mp
            self._hands = mp.solutions.hands.Hands(max_num_hands=1)

        무거운 초기화(가중치 로딩 등)는 반드시 여기서 한 번만 한다.
        infer() 안에서 매 프레임 하면 처리율이 무너진다.

        워커가 1개이므로 여기서 만든 객체는 한 스레드에서만 쓰인다.
        (MediaPipe Hands 처럼 스레드 안전하지 않은 객체도 그대로 써도 된다)
        """
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self._model, self._labels, self._num_frames = load_signal_model(MODEL_PATH, self._device)
        self._extractor = FeatureExtractor()
        self._window = deque(maxlen=self._num_frames)
        self._stabilizer = SignalStabilizer(threshold=0.8, consecutive=5, ema=0.4, release_grace=1.0)
        self._t0 = time.monotonic()

        self._hud = Hud()
        # infer()(추론 스레드) -> show()(렌더 스레드) 로 최신 렌더 데이터만 넘기는 한 칸 버퍼.
        # 캡처->추론 사이에 쓰던 것과 같은 클래스다 — "낡은 걸 버리고 최신만" 정책이 여기도 똑같이 맞다.
        self._render_buffer = LatestFrameBuffer()

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
    
        timestamp_ms = int((time.monotonic() - self._t0) * 1000)
        hand_result, pose_result = self._extractor.detect(frame, timestamp_ms)
        draw_detections(frame, hand_result, pose_result)   # 렌더용 — 손/포즈 랜드마크를 프레임에 직접 그린다
        self._window.append(FeatureExtractor.vector(hand_result, pose_result))

        if len(self._window) < self._num_frames:
            self._push_render(frame)
            return None

        arr = np.stack(self._window)
        pose_ratio = float((arr[:, HAND_DIM * 2:] != 0).any(axis=1).mean())
        if pose_ratio < 0.3:
            self._stabilizer.mark_person_absent(is_blurred=False)
            self._push_render(frame)
            return None

        x = torch.from_numpy(arr[None]).to(self._device)
        with torch.no_grad():
            raw_probs = torch.softmax(self._model(x), dim=1)[0].cpu().numpy()
        self._stabilizer.update(raw_probs, self._labels, is_blurred=False)

        idx = self._stabilizer.confirmed_idx
        self._push_render(frame)
        if idx is None:
            return None

        return LABEL_MAP.get(self._labels[idx])

    def _push_render(self, frame):
        """
        렌더 스레드(show())에 넘길 최신 상태를 한 칸 버퍼에 넣는다. **추론 스레드에서 호출된다.**

        frame 은 위에서 draw_detections() 로 랜드마크가 이미 그려진 것이다 — 렌더 스레드가
        따로 다시 그릴 필요 없게, cv2 작업(draw_detections)도 추론 스레드에서 끝내 둔다.
        show() 쪽에는 Hud.render() 의 텍스트/패널 오버레이 + imshow/waitKey 만 남는다.
        """
        header_color = (0, 220, 0) if self._stabilizer.confirmed_idx is not None else (80, 80, 255)
        self._render_buffer.put_latest({
            'frame': frame,
            'labels': self._labels,
            'probs': self._stabilizer.probs,
            'top': self._stabilizer.top,
            'confirmed_idx': self._stabilizer.confirmed_idx,
            'threshold': self._stabilizer.threshold,
            'header': f'확정: {self._stabilizer.confirmed}',
            'header_color': header_color,
            'status': self._stabilizer.status,
        })

    def show(self):
        """
        렌더 스레드에서 폴링한다. 최신 결과가 있으면 그리고, q/ESC 여부를 반환한다.

        새 결과가 없어도 cv2.waitKey(1)는 반드시 부른다 — Qt/cv2 창은 waitKey로 이벤트
        루프를 계속 펌핑해 줘야 정상적으로 갱신·반응한다. 추론이 잠깐 밀려 결과가 안
        들어오는 동안 이 호출 자체를 건너뛰면(즉 waitKey가 한동안 하나도 안 불리면) 창이
        멈추거나 깨진 것처럼 보인다 — 마지막 imshow 이미지는 그대로지만, 그걸 화면에
        실제로 리페인트하는 건 waitKey 쪽 이벤트 처리가 담당하기 때문이다.
        """
        payload = self._render_buffer.get(timeout=0.03)
        if payload is None:
            key = cv2.waitKey(1) & 0xFF
            return key in (ord('q'), 27)
        return self._hud.render(
            payload['frame'], payload['labels'], payload['probs'], payload['top'],
            payload['confirmed_idx'], payload['threshold'],
            header=payload['header'], header_color=payload['header_color'],
            status=payload['status'])

    def close(self):
        self._hud.close()
