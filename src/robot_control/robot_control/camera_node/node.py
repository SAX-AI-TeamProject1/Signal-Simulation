"""
조립자 — 네 부품을 배선하고 타이머/스레드만 관리한다.

이 파일은 cv2 도 모델도 Twist 도 직접 다루지 않는다. 그래서 import 목록에
cv2 가 없다. 배선만 하는 파일이라는 사실이 맨 위에서 바로 보인다.

두 개의 실행 흐름:

    [실행기 스레드]  1/fps 마다 _on_timer
        FrameSource.read() → ImagePublisher.publish() → LatestFrameQueue.put_latest()

    [워커 스레드 1개]  _infer_loop
        LatestFrameQueue.get() → GestureInference.infer() → GestureCommandPublisher.publish()

워커를 1개로 고정한 이유: 2개 이상이면 추론 시간 편차 때문에 완료 순서가 뒤바뀐다.
라벨은 이벤트가 아니라 상태(STOP/FORWARD/...)라서, 낡은 라벨이 최신 라벨을 덮으면
STOP 다음에 FORWARD 가 나가는 사고가 된다.

발행: image_webcam (sensor_msgs/Image, bgr8), gesture (std_msgs/String),
      cmd_vel_gesture (geometry_msgs/Twist)

실행:
    ros2 run robot_control camera_node
    ros2 run robot_control camera_node --ros-args -p device_id:=1
    ros2 run robot_control camera_node --ros-args -p enable_inference:=false
"""

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_control.camera_node.command_publisher import GestureCommandPublisher
from robot_control.camera_node.frame_source import FrameSource
from robot_control.camera_node.image_publisher import ImagePublisher
from robot_control.camera_node.inference import GestureInference
from robot_control.camera_node.shutdown import is_shutting_down
from robot_control.camera_node.swap_frame import LatestFrameBuffer
from robot_control.metrics import InferenceMetrics


class CameraNode(Node):
    """네 부품을 조립하고 두 개의 실행 흐름을 관리한다."""

    # 워커가 종료 플래그를 확인하러 깨어나는 주기(초).
    WORKER_POLL_SEC = 0.1
    # 종료 시 진행 중인 infer() 한 번이 끝나기를 기다려 주는 시간(초).
    WORKER_JOIN_TIMEOUT_SEC = 2.0

    def __init__(self):
        """파라미터를 읽고, 부품 넷을 만들고, 워커와 타이머를 띄운다."""
        super().__init__('camera_node')

        params = self._declare_and_read_parameters()

        # **종료 신호. 네 부품이 공유한다(누구든 컨텍스트가 죽은 걸 발견하면 set).**
        self._stop_event = threading.Event()

        # 통계 수집은 robot_control/metrics.py 로 분리돼 있다(이 패키지 밖).
        # inline 방식(지금)과 split 방식(별도 AI 노드)의 수치를 같은 자로 재려면
        # 양쪽이 같은 클래스를 써야 한다.
        # 스레드 규칙(어느 카운터를 어느 스레드가 건드리는지)은 그쪽 모듈 주석 참고.
        self._metrics = InferenceMetrics()

        # ── 부품 1: 하드웨어 캡처 (frame_source.py) ─────────────────────
        self._source = FrameSource(
            device_id=params['device_id'],
            width=params['frame_width'],
            height=params['frame_height'],
            fps=params['fps'],
            backend=params['capture_backend'])  # 어차피 linux 환경에서 할 거라 v4l2 고정

        # 카메라가 요청 해상도를 거절하는 일은 흔하다. 조용히 넘어가면 나중에
        # "모델 입력 크기가 왜 다르지?" 로 시간을 버린다 → 여기서 바로 알린다.
        # (FrameSource 는 로그를 찍지 않으므로 판단은 조립자인 여기가 한다)
        if self._source.actual_size != self._source.requested_size:
            req_w, req_h = self._source.requested_size
            act_w, act_h = self._source.actual_size
            self.get_logger().warn(
                f'카메라가 요청 해상도를 거절했습니다: 요청 {req_w}x{req_h} → '
                f'실제 {act_w}x{act_h}. 모델 입력 크기를 실제값 기준으로 맞추세요.')

        # ── 부품 2: 이미지 발행 (image_publisher.py) ────────────────────
        # 하지만, 이 퍼블리셔는 안 쓸거 같음.
        # 추후에 pub해서 다른 프로세스에 render하게 되는 경우에 쓰기 위해 존재
        self._image_pub = ImagePublisher(
            node=self,
            stop_event=self._stop_event,
            frame_id=params['frame_id'],
            enabled=params['publish_image'])

        # ── 부품 4: 라벨 → Twist (command_publisher.py) ─────────────────
        # (3번보다 먼저 만든다. 3번이 실패해도 4번 퍼블리셔는 살아 있어야
        #  토픽 그래프가 정상으로 보이기 때문)
        self._command_pub = GestureCommandPublisher(
            node=self,
            stop_event=self._stop_event,
            linear_speed=params['linear_speed'],
            angular_speed=params['angular_speed'])

        # ── 부품 3: 추론 (inference.py) + 워커 스레드 ───────────────────
        self._swap_buffer = LatestFrameBuffer()
        self._inference = None
        self._infer_thread = None
        self._render_thread = None
        if params['enable_inference']:
            self._inference = self._build_inference()
            self._inference.load_model()    # 워커 시작 전에 1회 (무거운 초기화)
            # 워커는 정확히 1개. 늘리면 완료 순서가 뒤바뀐다(파일 상단 설명 참고).
            # daemon=True: 메인이 끝날 때 이 스레드가 남아 있어도 같이 죽는다.
            #              False 면 무한 루프인 워커 때문에 프로세스가 안 죽는다.
            self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
            self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
            self._infer_thread.start()
            self._render_thread.start()

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
        # 가능한 값은 frame_source.CAPTURE_BACKENDS 참고.
        self.declare_parameter('capture_backend', 'v4l2')
        # 추론을 끄면 순수 카메라 노드로 동작한다(카메라만 검증할 때 사용).
        self.declare_parameter('enable_inference', True)
        # 이미지 토픽 발행. 기본 off — 이유는 image_publisher.py 독스트링 참고.
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
        # (해상도/백엔드 검증은 FrameSource 가, 속도 검증은
        #  GestureCommandPublisher 가 자기 것으로 따로 한다)
        if params['fps'] <= 0.0:
            raise ValueError(f'fps 는 0 보다 커야 합니다 (받은 값: {params["fps"]})')
        return params

    def _build_inference(self):
        """
        추론 구현체를 만든다. 모델을 갈아끼우는 지점은 여기 한 곳뿐이다.

        외부 리포지토리 모델이 준비되면 이 한 줄만 바꾼다:
            return MyHandGesture(self.get_logger())
        """
        return GestureInference(self.get_logger())

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

        if self._swap_buffer.put_latest(frame):
            self._metrics.record_drop()


    # ------------------------------------------------------ 워커 스레드 (추론)

    def _render_loop(self):
        """워커 스레드 본체. 큐에서 최신 프레임을 꺼내 '렌더링'만 한다."""
        while not self._stop_event.is_set():
            self._inference.show()

    def _infer_loop(self):
        """워커 스레드 본체. 큐에서 최신 프레임을 꺼내 추론하고 라벨을 발행한다."""
        while not self._stop_event.is_set():
            frame = self._swap_buffer.get(timeout=self.WORKER_POLL_SEC)
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
