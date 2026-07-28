"""
camera_node 리팩토링본 — 네 가지 책임을 각각의 클래스로 분리했다.

원본 camera_node.py 는 한 클래스가 아래 네 가지를 전부 들고 있었다.
네 가지는 "바뀌는 이유"가 서로 다르다. 웹캠을 바꾸는 것과 모델을 바꾸는 것과
로봇 속도를 바꾸는 것은 아무 관계가 없는데, 한 파일 한 클래스에 섞여 있으면
하나를 고칠 때 나머지 셋을 같이 읽어야 한다. 그래서 분리했다.

    ┌────────────────────────────┬──────────────────────────────────────────┐
    │ 1. FrameSource             │ 하드웨어 캡처 (cv2.VideoCapture)          │
    │ 2. ImagePublisher          │ 이미지 발행 (sensor_msgs/Image)           │
    │ 3. GestureInference        │ 추론  ← 본래 외부 리포지토리 소관          │
    │ 4. GestureCommandPublisher │ 라벨→Twist  ← 본래 command_node 소관      │
    └────────────────────────────┴──────────────────────────────────────────┘

    CameraNode 는 위 넷을 "조립"하고 타이머/스레드만 관리한다.
    즉 CameraNode 자체는 카메라도 모르고 모델도 모른다. 배선만 한다.

분리해서 얻는 것:
    - 1,3,4 는 rclpy 를 쓰지 않는다 → ROS 없이 순수 파이썬으로 테스트할 수 있다.
    - 3 을 통째로 갈아끼우면 외부 리포지토리 모델이 그대로 붙는다(GestureInference 상속).
    - 3,4 를 별도 노드로 떼어낼 때(split 방식) 클래스를 그대로 옮기면 된다.
      doc/design.md 는 추론이 외부 리포지토리, config/twist_mux.yaml 은 라벨→Twist 가
      command_node 소관이라고 적고 있다. 지금은 성능(직렬화 비용) 때문에 한 노드에
      두지만, 경계는 클래스로 미리 그어 둔다.

스레드 구조는 원본과 같다(검증된 부분이라 유지):
    ROS 타이머 콜백 : 프레임 캡처 → 이미지 발행 → 큐에 넣기 (무거운 일 금지)
    워커 스레드 1개 : 큐에서 꺼내 infer() → 라벨/Twist 발행

    워커를 1개로 고정한 이유: 2개 이상이면 추론 시간 편차 때문에 완료 순서가 뒤바뀐다.
    라벨은 이벤트가 아니라 상태(STOP/FORWARD/...)라서, 낡은 라벨이 최신 라벨을 덮으면
    STOP 다음에 FORWARD 가 나가는 사고가 된다.

원본 대비 고친 것(구조 변경 외):
    - 큐 깊이 5 → 1. 주석은 "깊이 1"이라 적혀 있었는데 코드는 5였다. 깊이 5 면 최악
      5프레임(추론 50ms 기준 250ms) 지난 화면으로 판단한다. STOP 지연은 안전 문제다.
    - 매 프레임 numpy 배열 전체를 로그로 찍던 디버그 잔여물 제거. throttle 은 출력만
      막을 뿐 f-string 평가는 매번 일어나 초당 20회 0.9MB repr 을 만들고 버렸다.
    - cv2.CAP_PROP_BUFFERSIZE=1 추가. 이게 없으면 드라이버 버퍼에 프레임이 쌓여서
      애플리케이션 큐를 1로 줄여도 read() 가 낡은 프레임을 준다.
    - 미정의 라벨 로그 레벨 fatal → warn. 무시하고 계속 도는 상황에 fatal 은 과하다.
    - LABELS 를 LABEL_MOTION 에서 파생. 원본은 두 곳에 손으로 적고 "일치시켜라"는
      주석으로 사람 손에 맡기고 있었다.
    - 캡처 백엔드를 파라미터로. V4L2 하드코딩이라 Linux 밖에서는 열리지 않았다.
    - 요청 해상도와 실제 해상도가 다르면 경고. 원본은 로그에 찍기만 했다.

발행: image_webcam (sensor_msgs/Image, bgr8), gesture (std_msgs/String),
      cmd_vel_gesture (geometry_msgs/Twist)

실행:
    ros2 run robot_control camera_node_refactory
    ros2 run robot_control camera_node_refactory --ros-args -p device_id:=1
    ros2 run robot_control camera_node_refactory --ros-args -p enable_inference:=false
"""

import queue
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_control.metrics import InferenceMetrics
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# 공용 상수 — 라벨의 단일 출처(single source of truth)
#
# 추론(3번)은 이 라벨을 "만들고", 제어 매핑(4번)은 이 라벨을 "소비한다".
# 두 클래스가 같은 정의를 봐야 하므로 어느 한쪽 안에 두지 않고 모듈 최상단에 둔다.
# ─────────────────────────────────────────────────────────────────────────────

# 라벨 → (전진 배수, 회전 배수).
# 여기에는 "방향"만 두고 실제 속도는 파라미터로 곱한다.
# 방향은 로봇이 바뀌어도 그대로지만, 속도는 로봇마다 다르기 때문이다.
LABEL_MOTION = {
    'STOP':    (0.0, 0.0),
    'FORWARD': (1.0, 0.0),
    'LEFT':    (0.0, +1.0),   # angular.z 는 + 가 좌회전
    'RIGHT':   (0.0, -1.0),
}

# 발행이 허용된 라벨. LABEL_MOTION 에서 파생시켜 둘이 어긋날 수 없게 만든다.
# (원본은 두 곳에 손으로 적고 "키는 LABELS 와 일치해야 한다"는 주석으로 관리했다)
LABELS = tuple(LABEL_MOTION)

# OpenCV 캡처 백엔드 이름 → cv2 상수.
# V4L2 를 하드코딩하면 Linux 밖에서 열리지 않는다. 파라미터로 고를 수 있게 한다.
CAPTURE_BACKENDS = {
    'v4l2': cv2.CAP_V4L2,           # Linux (기본, 실제 배포 환경)
    'any': cv2.CAP_ANY,             # OpenCV 가 알아서 고름
    'avfoundation': cv2.CAP_AVFOUNDATION,   # macOS
    'dshow': cv2.CAP_DSHOW,         # Windows
}


# ─────────────────────────────────────────────────────────────────────────────
# 1. 하드웨어 캡처
# ─────────────────────────────────────────────────────────────────────────────

class FrameSource:
    """
    웹캠 장치 하나를 열고 프레임을 읽어 준다. cv2.VideoCapture 의 얇은 래퍼.

    이 클래스는 rclpy 를 import 하지 않는다. 일부러 그렇다.
    - ROS 없이 단독으로 돌려볼 수 있다(카메라만 따로 검증할 때).
    - 나중에 파일/시뮬레이션 영상 소스로 갈아끼울 때 이 클래스만 바꾸면 된다.

    로그도 직접 찍지 않는다. 상태는 프로퍼티로 노출만 하고, 무엇을 경고할지는
    호출자(CameraNode)가 정한다. 로거를 주입받으면 다시 ROS 에 묶이기 때문이다.
    """

    def __init__(self, device_id, width, height, fps, backend):
        """
        장치를 열고 해상도/FPS 를 설정한다. 열지 못하면 RuntimeError.

        인자:
            device_id: /dev/video<N> 의 N
            width, height: 요청 해상도(카메라가 거절할 수 있다 → actual_size 로 확인)
            fps: 요청 프레임레이트(마찬가지로 카메라가 거절할 수 있다)
            backend: CAPTURE_BACKENDS 의 키 문자열
        """
        if width <= 0 or height <= 0:
            raise ValueError(f'해상도는 0 보다 커야 합니다 (받은 값: {width}x{height})')
        if backend not in CAPTURE_BACKENDS:
            raise ValueError(
                f'알 수 없는 캡처 백엔드: {backend!r} (가능: {list(CAPTURE_BACKENDS)})')

        self._device_id = device_id
        self._requested_size = (width, height)

        # 백엔드를 명시하는 이유: 지정하지 않으면 OpenCV 가 다른 백엔드를 잡아
        # 해상도/FPS 설정이 조용히 무시되는 경우가 있다.
        self._cap = cv2.VideoCapture(device_id, CAPTURE_BACKENDS[backend])
        if not self._cap.isOpened():
            raise RuntimeError(
                f'웹캠을 열지 못했습니다: /dev/video{device_id} (backend={backend}) — '
                '장치 존재 여부(ls /dev/video*)와 권한(video 그룹)을 확인하세요.')

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        # 드라이버 내부 버퍼를 1장으로 제한한다. 이게 없으면:
        #   카메라가 30fps 로 채우는데 우리가 20Hz 로 읽으면 초당 10장이 버퍼에 쌓이고,
        #   read() 는 "가장 오래된" 프레임부터 돌려준다 → 시간이 갈수록 화면이 밀린다.
        # 애플리케이션 큐를 1로 줄여도 이걸 안 하면 드라이버 단에서 지연이 남는다.
        # (백엔드가 지원하지 않으면 조용히 무시된다 — 설정해서 손해 볼 일은 없다)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    @property
    def device_id(self):
        """열려 있는 장치 번호."""
        return self._device_id

    @property
    def requested_size(self):
        """요청했던 (width, height)."""
        return self._requested_size

    @property
    def actual_size(self):
        """카메라가 실제로 적용한 (width, height). 요청과 다를 수 있다."""
        if self._cap is None:
            return (0, 0)
        return (int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    @property
    def is_open(self):
        """아직 장치를 들고 있으면 True. release() 후에는 False."""
        return self._cap is not None

    def read(self):
        """
        프레임 한 장을 읽는다.

        반환:
            numpy.ndarray (H, W, 3) uint8 BGR — 성공
            None — 실패했거나 이미 release() 된 상태

        주의: 이 호출은 블로킹이다(다음 프레임이 올 때까지 최대 1/fps 만큼 멈춘다).
              그래서 호출자는 이걸 실행기 스레드에서 부르되 그 뒤에 무거운 일을
              얹지 않아야 한다.
        """
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        # cv2 의 read() 는 호출마다 새 배열을 반환한다. 그래서 이 프레임을 워커에게
        # 넘겨도 다음 read() 가 덮어쓰지 않는다(별도 복사 불필요).
        return frame if ok else None

    def release(self):
        """장치를 반납한다. 여러 번 불러도 안전하다."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None


# ─────────────────────────────────────────────────────────────────────────────
# 2. 이미지 발행
# ─────────────────────────────────────────────────────────────────────────────

class ImagePublisher:
    """
    프레임을 sensor_msgs/Image 로 발행한다.

    기본은 꺼져 있다(enabled=False). 이 노드는 추론을 내부 큐로 넘기므로
    image_webcam 을 구독하는 노드가 평소에 없는데, 구독자가 0명이어도
    cv2_to_imgmsg 변환 비용(640x480x3 = 0.9MB, 20fps 면 초당 18MB 복사)은
    그대로 나가기 때문이다.
    rosbag2 녹화나 rviz2 확인이 필요할 때만 켠다.

    QoS 는 qos_profile_sensor_data(BEST_EFFORT).
    영상은 최신 프레임이 중요하고 낡은 프레임 재전송은 의미가 없어서다.
    주의: 구독 측(rviz2 등)이 RELIABLE 이면 에러 없이 조용히 매칭 실패한다.
          "토픽은 있는데 데이터가 안 온다"면 ros2 topic info --verbose 로 QoS 를 볼 것.
    """

    def __init__(self, node, stop_event, frame_id, enabled, topic='image_webcam'):
        """
        퍼블리셔를 만든다. enabled=False 여도 퍼블리셔 자체는 만들어 둔다.

        인자:
            node: 퍼블리셔를 소유할 rclpy 노드
            stop_event: 종료 신호(threading.Event). 발행 전에 확인하고,
                        발행 중 컨텍스트가 죽으면 여기에 set 해서 전체에 알린다.
            frame_id: Image 헤더의 frame_id (tf 프레임 이름)
            enabled: False 면 publish() 가 아무 일도 하지 않는다
        """
        self._node = node
        self._stop_event = stop_event
        self._frame_id = frame_id
        self._enabled = enabled
        self._bridge = CvBridge()
        self._pub = node.create_publisher(Image, topic, qos_profile_sensor_data)

    @property
    def enabled(self):
        """이미지 발행이 켜져 있으면 True."""
        return self._enabled

    def publish(self, frame):
        """
        프레임 한 장을 발행한다.

        반환:
            True  — 발행했거나, 꺼져 있어서 의도적으로 건너뛰었다
            False — 종료 중이라 발행하지 못했다(호출자는 이번 주기를 접어야 한다)
        """
        if not self._enabled:
            return True
        if is_shutting_down(self._stop_event):
            return False

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        try:
            self._pub.publish(msg)
        except RuntimeError:
            # 위 검사와 이 발행 사이에 종료가 끼어든 경우(InvalidHandle / RCLError).
            # RCLError 는 공개 import 경로가 없어(rclpy._rclpy_pybind11) RuntimeError 로 잡는다.
            self._stop_event.set()
            return False
        return True


# ─────────────────────────────────────────────────────────────────────────────
# 3. 추론  ← 본래 외부 리포지토리 소관
# ─────────────────────────────────────────────────────────────────────────────

class GestureInference:
    """
    프레임 → 수신호 라벨. 이 프로젝트의 "인지(perception)" 경계면이다.

    doc/design.md: 카메라 기반 이미지 프로세싱은 별도 리포지토리에서 개발되며,
    이 리포지토리는 처리된 인지 데이터를 "외부 입력"으로 취급해야 한다.
    그래서 이 클래스는 알고리즘을 담지 않고 **인터페이스만** 정의한다.
    실제 모델은 이 클래스를 상속해서 붙인다.

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
    │  그리고 CameraNode._build_inference() 가 이걸 만들게 바꾼다.        │
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
            LABELS 중 하나의 문자열 → gesture / cmd_vel_gesture 로 발행된다.
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


# ─────────────────────────────────────────────────────────────────────────────
# 4. 라벨 → Twist 제어 매핑  ← 본래 command_node 소관
# ─────────────────────────────────────────────────────────────────────────────

class GestureCommandPublisher:
    """
    라벨 문자열을 받아 두 토픽으로 내보낸다.

        gesture (std_msgs/String)          — 관측·기록용.
            rosbag 에 남겨야 "언제 무슨 수신호였나"를 알 수 있고,
            "인식→반응 지연시간" 측정도 이 타임스탬프가 있어야 한다.

        cmd_vel_gesture (geometry_msgs/Twist) — 제어용.
            twist_mux 의 gesture 입력(priority 50)으로 들어간다.
            이 노드는 네임스페이스가 없어 그냥 두면 /cmd_vel_gesture 가 되는데
            twist_mux 는 /robot1/cmd_vel_gesture 를 구독한다 → launch 에서 리맵.

    config/twist_mux.yaml 은 이 변환을 별도 command_node 가 맡는 것으로 적고 있다.
    지금 한 노드 안에 두는 이유는 변환이 딕셔너리 조회 한 번이라 노드를 하나 더
    띄울 만큼의 일이 아니어서다. 다만 클래스로는 분리해 두어, 나중에 노드로 떼어낼
    때 이 파일에서 그대로 들어내면 되게 했다.
    """

    def __init__(self, node, stop_event, linear_speed, angular_speed,
                 gesture_topic='gesture', cmd_vel_topic='cmd_vel_gesture'):
        """
        퍼블리셔 2개를 만들고, 라벨별 Twist 를 미리 만들어 캐싱한다.

        인자:
            linear_speed: FORWARD 전진 속도 (m/s)
            angular_speed: LEFT/RIGHT 회전 속도 (rad/s)

        Twist 를 미리 만드는 이유: 라벨이 4개뿐이라 매 프레임 새로 만들 이유가 없다.
        캐시된 객체를 재발행해도 되는 것은 publish() 가 내부에서 직렬화하기 때문이다
        (발행 후 그 객체를 고치지만 않으면 된다 → 이 클래스 밖으로 내보내지 않는다).
        """
        if linear_speed < 0.0 or angular_speed < 0.0:
            raise ValueError(
                f'속도는 음수일 수 없습니다 (linear={linear_speed}, angular={angular_speed}). '
                '방향은 LABEL_MOTION 의 부호가 정합니다.')

        self._node = node
        self._stop_event = stop_event
        self._last_label = None     # 워커 스레드에서만 읽고 쓴다 → 락 불필요

        # QoS 는 depth 10 + 기본 RELIABLE. 영상과 달리 라벨은 유실되면 안 된다
        # (특히 STOP). 그래서 BEST_EFFORT 를 쓰지 않는다.
        self._pub_gesture = node.create_publisher(String, gesture_topic, 10)
        self._pub_cmd_vel = node.create_publisher(Twist, cmd_vel_topic, 10)

        self._label_to_twist = {}
        for label, (lin_mul, ang_mul) in LABEL_MOTION.items():
            twist = Twist()
            twist.linear.x = linear_speed * lin_mul
            twist.angular.z = angular_speed * ang_mul
            self._label_to_twist[label] = twist

    def publish(self, label):
        """
        라벨 하나를 발행한다. 워커 스레드에서 호출된다.

        (rclpy 퍼블리셔는 스레드 안전하므로 워커에서 바로 발행해도 된다)

        인자:
            label: LABELS 중 하나. None 이거나 목록에 없으면 발행하지 않는다.

        반환:
            True  — 발행했거나, 발행할 것이 없어 건너뛰었다
            False — 종료 중이다(호출자는 워커 루프를 빠져나와야 한다)

        ┌─ 미해결 설계 이슈 (팀 결정 필요) ────────────────────────────────┐
        │ twist_mux 의 gesture timeout 은 0.5초다. label 이 None 인 동안은  │
        │ 여기서 아무것도 발행하지 않으므로, 인식이 0.5초 넘게 끊기면        │
        │ twist_mux 가 gesture 소스를 버리고 속도 0 으로 본다.              │
        │ → 손이 잠깐 흔들려도 FORWARD 중이던 로봇이 덜컥거린다.            │
        │                                                                   │
        │ 해결하려면 마지막 라벨을 일정 시간 유지(hold/latch)해야 하는데,    │
        │ "STOP 은 오래 유지하고 FORWARD 는 빨리 놓는" 비대칭 정책이 안전상  │
        │ 맞을 수 있다. 이건 동작을 바꾸는 결정이라 리팩토링 범위 밖으로     │
        │ 두었다. 정책이 정해지면 이 클래스 안에서 처리하면 된다.            │
        └───────────────────────────────────────────────────────────────────┘
        """
        if label is None:
            return True
        if label not in LABELS:
            # fatal 이 아니라 warn: 무시하고 계속 도는 상황이라 치명적이지 않다.
            # 모델이 오타 난 라벨을 뱉는 흔한 실수라서 눈에는 띄어야 한다.
            self._node.get_logger().warn(
                f'정의되지 않은 라벨이라 무시합니다: {label!r} (허용: {LABELS})')
            return True

        # infer() 가 도는 사이에 종료가 시작됐을 수 있다 → 발행 직전에 확인.
        if is_shutting_down(self._stop_event):
            return False

        try:
            # 라벨이 바뀔 때만 로그. 매 프레임 찍으면 초당 20줄이 쏟아진다.
            if label != self._last_label:
                self._node.get_logger().info(f'수신호 인식: {label}')
                self._last_label = label
            self._pub_gesture.publish(String(data=label))
            self._pub_cmd_vel.publish(self._label_to_twist[label])
        except RuntimeError:
            # 위 검사와 발행 사이에 종료가 끼어든 경우.
            self._stop_event.set()
            return False
        return True


# ─────────────────────────────────────────────────────────────────────────────
# 조립용 부품 — 책임이라기보다 배관(plumbing)
# ─────────────────────────────────────────────────────────────────────────────

def is_shutting_down(stop_event):
    """
    종료가 시작됐는지 판단한다.

    Ctrl-C 는 rclpy 컨텍스트를 먼저 무효화한 뒤 spin() 을 빠져나온다.
    그래서 콜백/워커가 실행 중이면 이미 죽은 컨텍스트로 발행을 시도하게 되고
    RCLError("publisher's context is invalid") 가 난다. 발행 전에 이걸로 거른다.

    두 조건을 OR 로 보는 이유:
        stop_event  — 우리가 스스로 내린 종료 결정(destroy_node, 발행 실패)
        rclpy.ok()  — 바깥(시그널/launch)에서 내려온 종료
    """
    return stop_event.is_set() or not rclpy.ok()


class LatestFrameQueue:
    """
    캡처 스레드 → 워커 스레드로 프레임을 넘기는 깊이 1 큐.

    깊이 1 = 항상 최신 프레임만. 왜 이래야 하는지:

        무한 큐:  캡처 30/초, 추론 20/초 → 초당 10장씩 쌓임
                  1분 뒤 600장 대기 = 30초 전 화면으로 판단(메모리도 터진다)   ✗
        깊이 1 :  낡은 프레임을 버리고 최신으로 교체 → 지연이 누적되지 않는다   ✓

    로봇 제어에서 "30초 전 정지 신호"는 "놓친 프레임"보다 훨씬 위험하다.
    버리는 것은 사고가 아니라 설계다. 다만 얼마나 버리는지는 세어 둔다(metrics).
    """

    def __init__(self):
        """깊이 1 큐를 만든다."""
        self._queue = queue.Queue(maxsize=1)

    def put_latest(self, frame):
        """
        프레임을 넣는다. 자리가 없으면 안에 있던 낡은 프레임을 버리고 넣는다.

        반환:
            True  — 낡은 프레임을 한 장 버렸다(호출자가 유실로 집계)
            False — 버린 것 없이 그냥 들어갔다

        nowait 계열만 쓰는 이유: 그냥 put() 이면 자리가 날 때까지 블로킹되어
        타이머 콜백이 멈춘다. 캡처 스레드는 절대 기다리면 안 된다.
        """
        try:
            self._queue.put_nowait(frame)
            return False
        except queue.Full:
            pass

        # 여기 오는 경우: 워커가 아직 이전 프레임을 처리 중이다.
        # 생산자가 캡처 스레드 하나뿐이라 get 직후의 put 은 실패하지 않지만,
        # 워커가 그 찰나에 get 해 갈 수도 있으므로 예외는 그대로 흘려보낸다.
        try:
            self._queue.get_nowait()        # 낡은 프레임 폐기
            self._queue.put_nowait(frame)   # 최신 프레임 투입
        except (queue.Empty, queue.Full):
            pass
        return True

    def get(self, timeout):
        """
        프레임을 꺼낸다. timeout 초 안에 없으면 None.

        타임아웃을 두는 이유: 프레임이 안 와도 워커가 주기적으로 깨어나
        종료 플래그를 확인해야 노드가 종료 시 매달리지 않는다.
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None


# ─────────────────────────────────────────────────────────────────────────────
# 조립자 — 위 부품들을 배선하고 타이머/스레드만 관리한다
# ─────────────────────────────────────────────────────────────────────────────

class CameraNode(Node):
    """
    네 부품을 조립하고 두 개의 실행 흐름을 관리한다.

        [실행기 스레드]  1/fps 마다 _on_timer
            FrameSource.read() → ImagePublisher.publish() → LatestFrameQueue.put_latest()

        [워커 스레드 1개]  _infer_loop
            LatestFrameQueue.get() → GestureInference.infer() → GestureCommandPublisher.publish()

    이 클래스 자체는 cv2 도 모델도 Twist 도 직접 다루지 않는다. 배선만 한다.
    """

    # 워커가 종료 플래그를 확인하러 깨어나는 주기(초).
    WORKER_POLL_SEC = 0.1
    # 종료 시 진행 중인 infer() 한 번이 끝나기를 기다려 주는 시간(초).
    WORKER_JOIN_TIMEOUT_SEC = 2.0

    def __init__(self):
        """파라미터를 읽고, 부품 넷을 만들고, 워커와 타이머를 띄운다."""
        super().__init__('camera_node')

        params = self._declare_and_read_parameters()

        # 종료 신호. 네 부품이 공유한다(누구든 컨텍스트가 죽은 걸 발견하면 set).
        self._stop_event = threading.Event()

        # 통계 수집은 metrics.py 로 분리돼 있다. inline 방식(지금)과 split 방식
        # (별도 AI 노드)의 수치를 같은 자로 재려면 양쪽이 같은 클래스를 써야 한다.
        # 스레드 규칙(어느 카운터를 어느 스레드가 건드리는지)은 그쪽 모듈 주석 참고.
        self._metrics = InferenceMetrics()

        # ── 부품 1: 하드웨어 캡처 ────────────────────────────────────────
        self._source = FrameSource(
            device_id=params['device_id'],
            width=params['frame_width'],
            height=params['frame_height'],
            fps=params['fps'],
            backend=params['capture_backend'])
        # 카메라가 요청 해상도를 거절하는 일은 흔하다. 조용히 넘어가면 나중에
        # "모델 입력 크기가 왜 다르지?" 로 시간을 버린다 → 여기서 바로 알린다.
        if self._source.actual_size != self._source.requested_size:
            req_w, req_h = self._source.requested_size
            act_w, act_h = self._source.actual_size
            self.get_logger().warn(
                f'카메라가 요청 해상도를 거절했습니다: 요청 {req_w}x{req_h} → '
                f'실제 {act_w}x{act_h}. 모델 입력 크기를 실제값 기준으로 맞추세요.')

        # ── 부품 2: 이미지 발행 ──────────────────────────────────────────
        self._image_pub = ImagePublisher(
            node=self,
            stop_event=self._stop_event,
            frame_id=params['frame_id'],
            enabled=params['publish_image'])

        # ── 부품 4: 라벨 → Twist ─────────────────────────────────────────
        # (3번보다 먼저 만든다. 3번이 실패해도 4번 퍼블리셔는 살아 있어야
        #  토픽 그래프가 정상으로 보이기 때문)
        self._command_pub = GestureCommandPublisher(
            node=self,
            stop_event=self._stop_event,
            linear_speed=params['linear_speed'],
            angular_speed=params['angular_speed'])

        # ── 부품 3: 추론 + 워커 스레드 ───────────────────────────────────
        self._queue = LatestFrameQueue()
        self._inference = None
        self._infer_thread = None
        if params['enable_inference']:
            self._inference = self._build_inference()
            self._inference.load_model()    # 워커 시작 전에 1회 (무거운 초기화)
            # 워커는 정확히 1개. 늘리면 완료 순서가 뒤바뀐다(파일 상단 설명 참고).
            # daemon=True: 메인이 끝날 때 이 스레드가 남아 있어도 같이 죽는다.
            #              False 면 무한 루프인 워커 때문에 프로세스가 안 죽는다.
            self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
            self._infer_thread.start()

        # ── 타이머 2개 ───────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / params['fps'], self._on_timer)
        if params['stats_period'] > 0.0:
            # 통계 flush 타이머. 매 프레임 찍으면 로그가 폭발하니 모아서 한 줄로 낸다.
            # stats_period=1.0 이면 출력 숫자가 그대로 Hz 로 읽힌다.
            self.create_timer(params['stats_period'], self._log_stats)

        act_w, act_h = self._source.actual_size
        self.get_logger().info(
            f'웹캠 시작: /dev/video{self._source.device_id} {act_w}x{act_h} '
            f'@{params["fps"]}Hz (추론 {"on" if self._inference else "off"}, '
            f'이미지 발행 {"on" if self._image_pub.enabled else "off"})')

    # ------------------------------------------------------------------ 설정

    def _declare_and_read_parameters(self):
        """
        파라미터를 선언하고 값을 검증해서 dict 로 돌려준다.

        생성자에서 한 번만 읽으므로 런타임 변경은 반영되지 않는다.
            ros2 param set /camera_node fps 15.0    # ← 값은 바뀌지만 동작은 그대로
        동적 반영이 필요하면 add_on_set_parameters_callback 을 붙여야 한다.
        (타이머 재생성과 카메라 재설정이 얽혀 있어 지금은 하지 않았다)
        """
        self.declare_parameter('device_id', 0)              # /dev/video<N> 의 N
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        # 캡처/발행 주기(Hz). 추론 처리율보다 높으면 그 차이만큼 프레임을 버린다.
        # 추론 50ms 기준: 처리 한계 20장/초.  fps 30 → 유실 10/초(33%),  fps 20 → 유실 0.
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('frame_id', 'webcam')        # Image 헤더의 frame_id
        # 캡처 백엔드. 실배포는 Linux 라 v4l2 가 기본. 개발 머신에 맞춰 바꾼다.
        self.declare_parameter('capture_backend', 'v4l2')
        # 추론을 끄면 순수 카메라 노드로 동작한다(카메라만 검증할 때 사용).
        self.declare_parameter('enable_inference', True)
        # 이미지 토픽 발행. 기본 off — 이유는 ImagePublisher 독스트링 참고.
        self.declare_parameter('publish_image', False)
        # 처리량/유실 통계를 몇 초마다 찍을지. 0 이하면 끔.
        self.declare_parameter('stats_period', 1.0)
        # 라벨을 Twist 로 바꿀 때 쓸 속도. 로봇이 바뀌면 이 두 개만 조정하면 된다.
        self.declare_parameter('linear_speed', 0.2)         # FORWARD 전진 속도 (m/s)
        self.declare_parameter('angular_speed', 0.5)        # LEFT/RIGHT 회전 속도 (rad/s)

        names = ('device_id', 'frame_width', 'frame_height', 'fps', 'frame_id',
                 'capture_backend', 'enable_inference', 'publish_image',
                 'stats_period', 'linear_speed', 'angular_speed')
        params = {name: self.get_parameter(name).value for name in names}

        # 검증은 여기 한 곳에 모은다. 잘못된 값으로 카메라를 열고 나서 실패하면
        # 장치를 반납해야 하는 뒤처리가 생기므로, 열기 전에 전부 걸러낸다.
        # (해상도/백엔드 검증은 FrameSource 가 자기 것으로 따로 한다)
        if params['fps'] <= 0.0:
            raise ValueError(f'fps 는 0 보다 커야 합니다 (받은 값: {params["fps"]})')
        return params

    def _build_inference(self):
        """
        추론 구현체를 만든다. 모델을 갈아끼우는 지점은 여기 한 곳뿐이다.

        외부 리포지토리 모델이 준비되면 이 한 줄만 바꾼다:
            return MyHandGesture(self.get_logger())
        """
        return StubGestureInference(self.get_logger())

    # ------------------------------------------------- 실행기 스레드 (캡처)

    def _on_timer(self):
        """
        프레임 한 장 캡처 → 이미지 발행 → 추론 큐에 넣기.

        여기서 무거운 일을 하면 안 된다. spin() 은 단일 스레드라서 이 콜백이
        길어지면 통계 타이머와 (앞으로 추가될) 구독 콜백이 통째로 밀린다.
        """
        if not self._source.is_open or is_shutting_down(self._stop_event):
            return      # 종료 중(장치 반납됐거나 컨텍스트가 이미 내려감)

        frame = self._source.read()
        if frame is None:
            # read() 가 도는 동안 종료가 시작됐을 수 있다. 그 상태로 로그를 남기면
            # rcl 이 "Failed to publish log message to rosout" 를 찍는다 → 다시 확인.
            if not is_shutting_down(self._stop_event):
                self.get_logger().warn('프레임 읽기 실패 — 이번 주기는 건너뜁니다.',
                                       throttle_duration_sec=1.0)
            return
        self._metrics.record_capture()

        if not self._image_pub.publish(frame):
            return      # 종료 감지. 큐에 더 넣을 이유가 없다.

        if self._inference is None:
            return      # 추론 off — 순수 카메라 노드로 동작 중

        if self._queue.put_latest(frame):
            self._metrics.record_drop()

    # ------------------------------------------------------ 워커 스레드 (추론)

    def _infer_loop(self):
        """워커 스레드 본체. 큐에서 최신 프레임을 꺼내 추론하고 라벨을 발행한다."""
        while not self._stop_event.is_set():
            frame = self._queue.get(timeout=self.WORKER_POLL_SEC)
            if frame is None:
                continue    # 프레임이 안 왔다. 위에서 종료 플래그를 다시 확인한다.

            started = time.monotonic()
            try:
                label = self._inference.infer(frame)
            except Exception as exc:                                 # noqa: BLE001
                # 추론 예외로 스레드가 죽으면 이후 라벨이 영영 안 나간다 → 잡아서 계속 돈다.
                # (모델 코드는 외부 리포지토리 소관이라 어떤 예외가 올지 모른다)
                self.get_logger().error(f'infer() 예외: {exc}', throttle_duration_sec=1.0)
                continue
            # time.time() 이 아니라 monotonic: 시스템 시각이 뒤로 점프해도
            # 음수 소요시간이 나오지 않는다.
            self._metrics.record_inference((time.monotonic() - started) * 1000.0)

            if not self._command_pub.publish(label):
                break       # 종료 감지

    # ------------------------------------------------------------------ 통계

    def _log_stats(self):
        """
        캡처/추론/유실 통계를 한 줄로 남긴다. inline 방식과 split 방식 비교용.

        출력 예:
            [stats] 캡처=20 추론=19 유실=1 | 추론 median=52.3ms max=71.0ms
        """
        self.get_logger().info(f'[stats] {self._metrics.drain_report()}')

    # ------------------------------------------------------------------ 종료

    def destroy_node(self):
        """워커를 멈추고 카메라 장치를 반납한다. 부품의 역순으로 정리한다."""
        self._stop_event.set()

        if self._infer_thread is not None:
            # 진행 중인 infer() 한 번(느려도 수백 ms)이 끝날 여유를 준다.
            self._infer_thread.join(timeout=self.WORKER_JOIN_TIMEOUT_SEC)
            if self._infer_thread.is_alive():
                # daemon=True 라서 프로세스 종료를 막지는 않는다. 알리기만 한다.
                self.get_logger().warn('추론 워커가 제때 끝나지 않아 그대로 종료합니다.')
            self._infer_thread = None

        self._source.release()
        return super().destroy_node()


def main(args=None):
    """엔트리 포인트."""
    rclpy.init(args=args)
    node = None
    try:
        # 노드 생성 실패(카메라 못 엶 등)와 spin 중 종료를 구분해서 처리한다.
        # 둘을 한 try 로 묶으면 종료 시 나는 RCLError 까지 "카메라 오류"로 잘못 보고된다.
        try:
            node = CameraNode()
        except (RuntimeError, ValueError) as exc:
            # 카메라를 못 열었을 때 스택트레이스 대신 원인만 보여준다.
            rclpy.logging.get_logger('camera_node').fatal(str(exc))
            return

        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            # Ctrl-C 나 launch 종료 신호. 정상 종료 경로.
            pass
        except RuntimeError:
            # 종료 신호와 spin 이 겹치면 rclpy 내부에서 RCLError(RuntimeError 파생)가 올라온다.
            #   "failed to initialize wait set: the given context is not valid"
            # 이것도 정상 종료 경로이므로 traceback 을 남기지 않는다.
            pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
