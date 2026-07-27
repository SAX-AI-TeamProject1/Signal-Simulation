"""
웹캠 캡처 + 수신호 추론을 한 노드에서 처리한다 (inline 방식).

구조:
    ROS 타이머 콜백  : 프레임 캡처 → /image_webcam 발행 → 큐에 넣기만 (무거운 일 금지)
    워커 스레드 1개  : 큐에서 꺼내 infer() 실행 → /gesture 발행

큐는 maxsize=1 이고 가득 차면 낡은 프레임을 버리고 최신으로 교체한다.
추론이 캡처보다 느려도 항상 최신 프레임을 보게 되고, 지연이 누적되지 않는다.

워커를 1개로 고정한 이유: 2개 이상이면 추론 시간 편차 때문에 완료 순서가 뒤바뀐다.
라벨은 이벤트가 아니라 상태(STOP/FORWARD/...)라서, 낡은 라벨이 최신 라벨을 덮으면
STOP 다음에 FORWARD 가 나가는 사고가 된다.

이 파일에서 팀원이 채울 곳은 load_model() 과 infer() 두 함수뿐이다.

발행: image_webcam (sensor_msgs/Image, bgr8), gesture (std_msgs/String)

실행:
    ros2 run robot_control camera_node
    ros2 run robot_control camera_node --ros-args -p device_id:=1 -p enable_inference:=false
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


class CameraNode(Node):
    """웹캠을 읽어 이미지를 발행하고, 별도 스레드에서 수신호를 추론한다."""

    # 발행이 허용된 라벨. 핸드오프 §4 의 command_node 매핑과 1:1 로 맞춰져 있다.
    #   STOP → (0, 0) / FORWARD → (0.2, 0) / LEFT → (0, +0.5) / RIGHT → (0, -0.5)
    LABELS = ('STOP', 'FORWARD', 'LEFT', 'RIGHT')  # 어차피 static 변수

    # 라벨 → (전진 배수, 회전 배수). 여기에는 "방향"만 두고 실제 속도는 파라미터로 곱한다.
    # 방향은 로봇이 바뀌어도 그대로지만 속도는 로봇마다 다르기 때문이다.
    # 키는 LABELS 와 일치해야 한다.
    LABEL_MOTION = {
        'STOP':    (0.0, 0.0),
        'FORWARD': (1.0, 0.0),
        'LEFT':    (0.0, +1.0),   # angular.z 는 + 가 좌회전
        'RIGHT':   (0.0, -1.0),
    }

    def __init__(self):
        """카메라를 열고, 발행자를 만들고, 추론 워커를 띄운다."""
        super().__init__('camera_node')

        self.declare_parameter('device_id', 0)          # /dev/video<N> 의 N
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        # 캡처/발행 주기(Hz). 추론 처리율보다 높으면 그 차이만큼 프레임을 버린다.
        # 추론 시간 50ms 기준: fps 30 → 유실 10/초(33%),  fps 20 → 유실 0.
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('frame_id', 'webcam')    # Image 헤더의 frame_id
        # 추론을 끄면 순수 카메라 노드로 동작한다(카메라만 검증할 때 사용).
        self.declare_parameter('enable_inference', True)
        # 이미지 토픽 발행. 기본 off — 이 노드는 추론을 내부 큐로 넘기므로
        # /image_webcam 을 구독하는 노드가 없다. 구독자가 없어도 직렬화 비용은 그대로 나간다.
        # rosbag2 녹화나 rviz2 확인이 필요할 때만 켠다: -p publish_image:=true
        self.declare_parameter('publish_image', False)
        # 처리량/유실 통계를 몇 초마다 찍을지. 0 이하면 끔.
        self.declare_parameter('stats_period', 1.0)
        # 라벨을 Twist 로 바꿀 때 쓸 속도. 로봇이 바뀌면 이 두 개만 조정하면 된다.
        self.declare_parameter('linear_speed', 0.2)     # FORWARD 전진 속도 (m/s)
        self.declare_parameter('angular_speed', 0.5)    # LEFT/RIGHT 회전 속도 (rad/s)

        """
        ros2 param set /camera_node fps 15.0     # 다른 터미널에서, 노드 실행 중에
        node 런타임에 위 명령어를 수행하면 아래 get_param의 value가 바뀔 수 있다.
        하지만, 현재 생성자에서만 하기때문에 동적으로 바뀌지 않는다.
        """

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        self._frame_id = self.get_parameter('frame_id').value
        self._inference_on = self.get_parameter('enable_inference').value
        self._publish_image = self.get_parameter('publish_image').value
        stats_period = self.get_parameter('stats_period').value

        if fps <= 0.0:
            raise ValueError(f'fps 는 0 보다 커야 합니다 (받은 값: {fps})')

        # V4L2(Video4Linux2) 를 명시적으로 지정: 지정하지 않으면 OpenCV 가 다른 백엔드를 잡아
        # 해상도/FPS 설정이 무시되는 경우가 있다.
        self._cam_handle = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self._cam_handle.isOpened():
            raise RuntimeError(
                f'웹캠을 열지 못했습니다: /dev/video{device_id} — '
                '장치 존재 여부(ls /dev/video*)와 권한(video 그룹)을 확인하세요.')
        self._cam_handle.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cam_handle.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cam_handle.set(cv2.CAP_PROP_FPS, fps)
        # 카메라 인풋에 대해서, 내가 원하는 크기와 프레임을 설정

        # 내가 설정한 크기로 동작하는지 확인하기위한 변수 저장
        actual_w = int(self._cam_handle.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cam_handle.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self._bridge = CvBridge()
        self._last_label = None

        # 통계 수집은 metrics.py 로 분리했다. split 방식(별도 AI 노드)과 같은 클래스를 써야
        # 두 방식의 수치를 같은 기준으로 비교할 수 있다.
        # 스레드 규칙(어느 카운터를 어느 스레드가 건드리는지)은 그쪽 모듈 주석 참고.
        self._metrics = InferenceMetrics()

        # QoS 는 qos_profile_sensor_data(BEST_EFFORT).
        # 구독 측(rviz2 등)이 RELIABLE 이면 에러 없이 조용히 매칭 실패한다.
        self._pub_image = self.create_publisher(Image, 'image_webcam', qos_profile_sensor_data)
        # 라벨(String) — 관측·기록용. rosbag 에 남겨야 "언제 무슨 수신호였나"를 알 수 있고,
        # 핸드오프 §8 의 "인식→반응 지연시간" 도 이 타임스탬프가 있어야 잰다.
        self._pub_gesture = self.create_publisher(String, 'gesture', 10)
        # 속도 명령(Twist) — 제어용. twist_mux 의 gesture 입력으로 들어간다.
        # 이 노드는 네임스페이스가 없어서 그냥 두면 /cmd_vel_gesture 가 되는데,
        # twist_mux 는 /robot1/cmd_vel_gesture 를 구독한다 → launch 에서 리맵으로 이어준다.
        self._pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel_gesture', 10)

        # 라벨 → Twist 를 미리 만들어 둔다(라벨 4개뿐이라 매번 만들 이유가 없다).
        linear = self.get_parameter('linear_speed').value
        angular = self.get_parameter('angular_speed').value
        self._label_to_twist = {}
        for label, (lin_mul, ang_mul) in self.LABEL_MOTION.items():
            twist = Twist()
            twist.linear.x = linear * lin_mul
            twist.angular.z = angular * ang_mul
            self._label_to_twist[label] = twist

        # 큐 깊이 1 = 항상 최신 프레임만. 추론이 밀려도 낡은 프레임이 쌓이지 않는다.
        self._queue = queue.Queue(maxsize=5)
        self._stop = threading.Event()
        self._infer_thread = None

        if self._inference_on:  # 카메라 노드에서 추론을 하는경우
            self.load_model()
            # 워커는 정확히 1개. 늘리면 완료 순서가 뒤바뀐다(파일 상단 설명 참고).
            # 인자 deamon=False는 thread가 끝날때까지 프로세스(py)가 무한대기한다(프로세스 종료 불가)
            self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
            self._infer_thread.start()

        self._timer = self.create_timer(1.0 / fps, self._on_timer)
        if stats_period > 0.0:  # 로그 flush 타이머
            self.create_timer(stats_period, self._log_stats)

        self.get_logger().info(
            f'웹캠 시작: /dev/video{device_id} {actual_w}x{actual_h} @{fps}Hz '
            f'(추론 {"on" if self._inference_on else "off"}, '
            f'이미지 발행 {"on" if self._publish_image else "off"})')

    def _shutting_down(self):
        """
        종료가 시작됐는지 판단한다.

        Ctrl-C 는 rclpy 컨텍스트를 먼저 무효화한 뒤 spin() 을 빠져나온다.
        그래서 콜백/워커가 실행 중이면 이미 죽은 컨텍스트로 발행을 시도하게 되고
        RCLError('publisher's context is invalid') 가 난다. 발행 전에 이걸로 거른다.
        """
        return self._stop.is_set() or not rclpy.ok()

    def _on_timer(self):
        """프레임 한 장 캡처 → 이미지 발행 → 추론 큐에 넣기. 무거운 일은 하지 않는다."""
        if self._cam_handle is None or self._shutting_down():
            return          # 종료 중(장치 반납됐거나 컨텍스트가 이미 내려감)
        ok, frame = self._cam_handle.read()
        if not ok:
            # read() 가 도는 동안 종료가 시작됐을 수 있다. 그 상태로 로그를 남기면
            # rcl 이 "Failed to publish log message to rosout" 를 찍는다 → 다시 확인.
            if not self._shutting_down():
                self.get_logger().warn('프레임 읽기 실패 — 이번 주기는 건너뜁니다.',
                                       throttle_duration_sec=1.0)
            return
        self._metrics.record_capture()
        if not self._shutting_down():
            self.get_logger().info(f'frame Info: {frame}', throttle_duration_sec=1.0)

        if self._publish_image:  # 이미지를 다른 노드에 pub 하는 경우{기본은 False여서 수행x}
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            try:
                self._pub_image.publish(msg)
            except RuntimeError:
                # 위 검사와 발행 사이에 종료가 끼어든 경우(InvalidHandle/RCLError).
                self._stop.set()
                return

        if not self._inference_on:
            return

        # cv2 의 read() 는 호출마다 새 배열을 반환하므로 워커가 들고 있는 동안
        # 이 프레임이 덮어써질 걱정은 없다(별도 복사 불필요).
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(frame)
            except (queue.Empty, queue.Full):
                pass
            self._metrics.record_drop()  # 큐가 꽉 차서 못 넣는 경우 드랍(bounded_queue 라서)

    def _infer_loop(self):
        """워커 스레드. 큐에서 최신 프레임을 꺼내 추론하고 라벨을 발행한다."""
        while not self._stop.is_set():
            try:
                # 타임아웃을 두는 이유: 프레임이 안 와도 주기적으로 깨어나
                # 종료 플래그를 확인해야 노드가 매달리지 않는다.
                frame = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            started = time.monotonic()
            try:
                label = self.infer(frame)
            except Exception as exc:                                 # noqa: BLE001
                # 추론 예외로 스레드가 죽으면 이후 라벨이 영영 안 나간다 → 잡아서 계속 돈다.
                self.get_logger().error(f'infer() 예외: {exc}', throttle_duration_sec=1.0)
                continue
            self._metrics.record_inference((time.monotonic() - started) * 1000.0)

            # infer() 가 도는 사이에 종료가 시작됐을 수 있다 → 발행 직전에 다시 확인.
            if self._shutting_down():
                break

            if label is None:
                continue
            if label not in self.LABELS:
                self.get_logger().fatal(
                    f'정의되지 않은 라벨이라 무시합니다: {label!r} (허용: {self.LABELS})')
                continue

            try:
                if label != self._last_label:
                    self.get_logger().info(f'수신호 인식: {label}')
                    self._last_label = label
                # rclpy 퍼블리셔는 스레드 안전하므로 워커에서 바로 발행해도 된다.
                # 라벨 → Twist 변환은 딕셔너리 조회 한 번이라 별도 노드나 스레드가 필요 없다.
                self._pub_gesture.publish(String(data=label))
                self._pub_cmd_vel.publish(self._label_to_twist[label])
            except RuntimeError:
                # 위 검사와 발행 사이에 종료가 끼어든 경우.
                # rclpy 가 던지는 InvalidHandle / RCLError 가 둘 다 RuntimeError 파생인데
                # RCLError 는 공개 import 경로가 없어(rclpy._rclpy_pybind11) RuntimeError 로 잡는다.
                self._stop.set()
                break

    def _log_stats(self):
        # metric 측정을 위해 존재
        """캡처/추론/유실 통계를 남긴다. split 방식과 수치를 비교하기 위한 것."""
        self.get_logger().info(f'[stats] {self._metrics.drain_report()}')

    # ==================== 여기부터 팀원 구현 구간 ====================

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
        self.get_logger().warn(
            'load_model() 이 아직 비어 있습니다 — 추론 없이 대기만 합니다. '
            'camera_node.py 의 "팀원 구현 구간"을 채워주세요.')

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
        time.sleep(0.05)  # 50ms sleep 걸기 (추론 시간 흉내)
        self.get_logger().warn('Test infer Log', throttle_duration_sec=1.0)
        return None

    # ==================== 팀원 구현 구간 끝 ====================

    def destroy_node(self):
        """워커를 멈추고 카메라 장치를 반납한다."""
        self._stop.set()
        if self._infer_thread is not None:
            # infer() 한 번(느려도 수백 ms)이 끝날 여유를 준다.
            self._infer_thread.join(timeout=2.0)
            if self._infer_thread.is_alive():
                self.get_logger().warn('추론 워커가 제때 끝나지 않아 그대로 종료합니다.')
            self._infer_thread = None
        if self._cam_handle is not None:
            self._cam_handle.release()
            self._cam_handle = None
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
