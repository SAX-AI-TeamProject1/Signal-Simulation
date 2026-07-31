"""
조립자 — 네 부품을 배선하고 타이머/스레드만 관리한다.

이 파일은 cv2 도 모델도 Twist 도 직접 다루지 않는다. 그래서 import 목록에
cv2 가 없다. 배선만 하는 파일이라는 사실이 맨 위에서 바로 보인다.

두 개의 실행 흐름:

    [캡처+추론 워커 1개]  _capture_infer_loop
        FrameSource.read() → ImagePublisher.publish() → GestureInference.infer()
        → GestureCommandPublisher.publish()
        → take_render_payload() 로 결과를 꺼내 렌더 워커에게 넘기고 깨운다

    [렌더 워커 1개]  _render_loop
        _render_cv 에서 대기 → GestureInference.show(payload) → 다시 대기
        show() 가 True 를 주면(HUD 창에서 q/ESC) 노드 종료를 요청한다

(실행기 스레드에는 통계 타이머만 남는다. 캡처는 타이머가 아니라 위 워커가 한다)

캡처와 추론을 한 스레드에 둔 이유: 추론이 병목이라 둘을 나눠도 처리율이 안 오른다.
실측(추론 79ms 기준) — 분리 14.2회/초, 병합 14.1회/초. 나누면 스레드와 전달 버퍼만
늘고 얻는 게 없다. 블로킹 read() 가 루프를 카메라 속도에 묶어 주므로 주기 관리도 없다.

워커를 1개로 고정한 이유: 2개 이상이면 추론 시간 편차 때문에 완료 순서가 뒤바뀐다.
라벨은 이벤트가 아니라 상태(STOP/FORWARD/...)라서, 낡은 라벨이 최신 라벨을 덮으면
STOP 다음에 FORWARD 가 나가는 사고가 된다.

렌더를 추론 워커에서 분리한 이유: GUI 갱신이 느리면 추론 처리율이 그만큼 깎인다.
분리하되 **자유 실행이 아니라 신호 기반**이다 — 아무도 상태를 갱신하지 않았는데 다시
그려봐야 같은 화면이므로, 추론 주기에 한 번만 그린다.

두 워커가 상태를 공유하지 않는 이유(중요): 렌더가 추론 객체 내부를 직접 읽으면
레이스가 난다(자세한 것은 inference.py 의 RenderPayload 주석 참고). 그래서 추론이
끝난 뒤 결과를 **불변 묶음으로 떠서 넘기고**, 렌더는 그 묶음만 본다. 넘긴 뒤 추론은
그 프레임을 다시 건드리지 않으므로 렌더가 그리는 동안 겹쳐 돌아도 안전하다.

깨우는 시점이 infer() 뒤인데도 처리율이 안 깎이는 이유: notify 는 락을 μs 만 쥐고
놓으므로 추론 워커는 곧바로 다음 프레임으로 넘어간다. 렌더가 그리는 시간은 그
다음 infer() 안에 묻힌다.

발행: image_webcam (sensor_msgs/Image, bgr8), gesture (std_msgs/String),
      cmd_vel_gesture (geometry_msgs/Twist)

실행:
    ros2 run signal_vision camera_node
    ros2 run signal_vision camera_node --ros-args -p device_id:=1
    ros2 run signal_vision camera_node --ros-args -p enable_inference:=false
"""

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from signal_vision.camera_node.command_publisher import GestureCommandPublisher
from signal_vision.camera_node.frame_source import FrameSource
from signal_vision.camera_node.image_publisher import ImagePublisher
from signal_vision.camera_node.inference import GestureInference
from signal_vision.camera_node.shutdown import is_shutting_down
from signal_vision.metrics import InferenceMetrics


class CameraNode(Node):
    """네 부품을 조립하고 두 개의 실행 흐름을 관리한다."""

    # 워커가 종료 플래그를 확인하러 깨어나는 주기(초).
    WORKER_POLL_SEC = 0.1
    # 종료 시 진행 중인 infer() 한 번이 끝나기를 기다려 주는 시간(초).
    WORKER_JOIN_TIMEOUT_SEC = 2.0
    # 프레임 읽기에 실패했을 때 캡처+추론 워커가 쉬는 시간(초).
    # 타이머 시절에는 실패해도 다음 발화까지 저절로 쉬었지만, 이제는 루프라서
    # 장치가 빠지면(read 가 즉시 None) 이게 없으면 코어 하나를 100% 태운다.
    CAPTURE_RETRY_SEC = 0.1

    def __init__(self):
        """파라미터를 읽고, 부품 넷을 만들고, 워커와 타이머를 띄운다."""
        super().__init__('camera_node')

        params = self._declare_and_read_parameters()

        # **종료 신호. 네 부품이 공유한다(누구든 컨텍스트가 죽은 걸 발견하면 set).**
        self._stop_event = threading.Event()

        # 통계 수집은 signal_vision/metrics.py 로 분리돼 있다(camera_node 폴더 밖).
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
        self._inference = None
        self._worker_thread = None
        self._render_thread = None

        # 캡처+추론 워커 → 렌더 워커로 결과를 넘기는 한 칸짜리 슬롯 + 그 신호.
        #
        # 슬롯이 따로 필요한 이유: notify() 는 대기 중인 스레드가 없으면 그냥 사라진다.
        # 렌더가 아직 이전 화면을 그리는 중에 온 notify 를 놓치지 않으려면 "그릴 게
        # 밀려 있다"는 사실이 남아 있어야 한다. payload 자체가 그 표시를 겸한다
        # (None 이 아니면 그릴 게 있다).
        #
        # 한 칸인 이유: 렌더가 밀린 동안 추론이 3번 돌았어도 그릴 것은 최신 결과
        # 하나뿐이다. 낡은 payload 를 덮어써도 안전하다 — 렌더가 이미 꺼내 간 것은
        # 자기 지역 변수로 참조를 들고 있어서 살아 있다.
        self._render_cv = threading.Condition()
        self._render_payload = None
        if params['enable_inference']:
            self._inference = self._build_inference()
            self._inference.load_model()    # 워커 시작 전에 1회 (무거운 초기화)
            # daemon=True: 메인이 끝날 때 이 스레드가 남아 있어도 같이 죽는다.
            #              False 면 무한 루프인 워커 때문에 프로세스가 안 죽는다.
            self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
            self._render_thread.start()

        # ── 캡처+추론 워커 ──────────────────────────────────────────────
        # 워커는 정확히 1개. 늘리면 완료 순서가 뒤바뀐다(파일 상단 설명 참고).
        # 추론이 off 여도 띄운다 — 그래야 순수 카메라 노드로 동작한다.
        # 렌더 워커보다 나중에 시작한다: 반대면 첫 payload 를 받을 상대가 없다.
        self._worker_thread = threading.Thread(target=self._capture_infer_loop, daemon=True)
        self._worker_thread.start()

        # ── 타이머 ───────────────────────────────────────────────────────
        if params['stats_period'] > 0.0:
            # 통계 flush 타이머. 매 프레임 찍으면 로그가 폭발하니 모아서 한 줄로 낸다.
            # stats_period=1.0 이면 출력 숫자가 그대로 Hz 로 읽힌다.
            self.create_timer(params['stats_period'], self._log_stats)

        act_w, act_h = self._source.actual_size
        self.get_logger().info(
            f'웹캠 시작: /dev/video{self._source.device_id} {act_w}x{act_h} '
            f'@{params["fps"]}Hz 요청 (추론 {"on" if self._inference else "off"}, '
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
        # 카메라에 요청할 프레임레이트(CAP_PROP_FPS). 소프트웨어 주기가 아니다 —
        # 캡처는 타이머가 아니라 워커의 블로킹 read() 가 페이싱한다.
        #
        # 추론(약 79ms → 12.5회/초)보다 높게 두는 것이 맞다. 낮추면 처리율은 그대로인데
        # 프레임만 낡는다. 실측: fps 20 → 프레임 나이 73ms,  fps 30 → 18ms.
        # 카메라 하드웨어 상한이 30 이라 그 위로 올려도 30 으로 잘린다.
        self.declare_parameter('fps', 30.0)
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
        # (이 월드의 real-time factor가 한때 ~0.5%까지 떨어져서 속도값을 극단적으로
        # 올렸던 적이 있는데, 진짜 원인은 navi_factory.sdf의 물리 스텝 크기였다
        # — max_step_size를 0.001→0.01로 늘려 real-time factor≈1로 고쳤으니
        # 이제 정상적인 속도값을 쓰면 된다.)
        self.declare_parameter('linear_speed', 0.5)         # FORWARD 전진 속도 (m/s)
        self.declare_parameter('angular_speed', 0.8)         # LEFT/RIGHT 회전 속도 (rad/s)

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

    # ------------------------------------------------ 캡처+추론 워커

    def _render_loop(self):
        """렌더 워커 본체. 추론 워커가 깨울 때만 한 번 그리고 다시 잠든다."""
        while not self._stop_event.is_set():
            with self._render_cv:
                # wait_for 로 조건을 다시 검사하는 이유: notify 없이 깨어나는 경우
                # (spurious wakeup)에 그냥 그리면 안 되기 때문.
                # timeout 을 두는 이유: notify 를 한 번도 못 받아도 WORKER_POLL_SEC 마다
                # 깨어나 위의 종료 플래그를 다시 본다(추론이 죽어도 매달리지 않는다).
                self._render_cv.wait_for(
                    lambda: self._render_payload is not None or self._stop_event.is_set(),
                    timeout=self.WORKER_POLL_SEC)
                if self._stop_event.is_set():
                    break
                if self._render_payload is None:
                    # 타임아웃으로 깨어났다(= wait_for 가 조건 불만족으로 반환). 흔한 길이다:
                    # 슬롯은 바로 아래에서 렌더 자신이 비우므로, 다음 추론이 끝나기 전까지는
                    # 계속 None 이다. 추론 한 바퀴가 WORKER_POLL_SEC 보다 길거나(느린 모델)
                    # 카메라 프레임이 끊기면 매번 여기로 온다.
                    continue
                # 슬롯을 비우면서 참조를 가져온다. 이 지역 변수가 살아 있는 한
                # 추론이 슬롯을 덮어써도 이 payload 는 온전하다.
                payload, self._render_payload = self._render_payload, None

            # show() 는 락 **밖에서** 부른다. GUI 갱신은 느린데, 락을 쥔 채 그리면
            # 추론 워커가 notify 하려고 락을 기다리다 그만큼 멈춘다.
            try:
                quit_requested = self._inference.show(payload)
            except Exception as exc:                                 # noqa: BLE001
                # 렌더 예외로 이 스레드가 죽어도 추론/발행은 계속돼야 한다.
                # (GUI 코드는 외부 리포지토리 소관이라 어떤 예외가 올지 모른다)
                self.get_logger().error(f'show() 예외: {exc}', throttle_duration_sec=1.0)
                continue

            if quit_requested:
                # HUD 창에서 q/ESC 를 눌렀다. 창만 닫으면 창 없이 계속 도는 좀비가 되므로
                # 노드째 내린다. 발행이 끊기면 twist_mux 가 0.5초 뒤 gesture 소스를
                # 버리고 로봇을 세우므로, 이 종료 자체는 안전한 쪽으로 떨어진다.
                self.get_logger().info('HUD 창에서 q/ESC 입력 — 노드를 종료합니다.')
                self._request_shutdown()
                break

    def _request_shutdown(self):
        """워커 스레드에서 노드 종료를 요청한다."""
        self._stop_event.set()
        # stop_event 만으로는 부족하다. rclpy.spin() 은 그 플래그를 보지 않으므로
        # 컨텍스트를 내려야 spin 이 ExternalShutdownException 으로 빠져나오고,
        # main 의 finally 가 destroy_node 까지 돌린다.
        if rclpy.ok():
            rclpy.shutdown()

    def _capture_infer_loop(self):
        """
        워커 본체. 프레임을 읽어 그 자리에서 추론하고 라벨을 발행한다.

        주기를 관리하지 않는 이유: read() 가 다음 프레임까지 블로킹이라 이 루프가
        저절로 카메라 속도에 맞춰진다. 추론이 그보다 느리면 그만큼 드라이버가
        프레임을 버리는데, 그건 속도 차이라 어느 구조로도 못 막는다.

        이 루프는 카메라(30fps)보다 느리게(약 12.5회/초) 읽으므로, 드라이버 버퍼에
        프레임이 쌓이고 read() 는 그중 가장 오래된 것을 돌려준다. 즉 버퍼 장수가
        곧 지연이다 — 그 절충의 근거는 frame_source.py 의 BUFFERSIZE 주석에 있다.
        """
        while not self._stop_event.is_set():
            if not self._source.is_open or is_shutting_down(self._stop_event):
                break       # 종료 중(장치 반납됐거나 컨텍스트가 이미 내려감)

            frame = self._source.read()
            if frame is None:
                # read() 가 도는 동안 종료가 시작됐을 수 있다. 그 상태로 로그를 남기면
                # rcl 이 "Failed to publish log message to rosout" 를 찍는다 → 다시 확인.
                if is_shutting_down(self._stop_event):
                    break
                self.get_logger().warn('프레임 읽기 실패 — 잠시 뒤 다시 시도합니다.',
                                       throttle_duration_sec=1.0)
                # sleep 이 아니라 Event.wait: 종료 신호가 오면 즉시 깨어난다.
                self._stop_event.wait(self.CAPTURE_RETRY_SEC)
                continue
            self._metrics.record_capture()

            if not self._image_pub.publish(frame):
                break       # 종료 감지

            if self._inference is None:
                continue    # 추론 off — 순수 카메라 노드로 동작 중

            started = time.monotonic()
            try:
                label = self._inference.infer(frame)
            except Exception as exc:                                 # noqa: BLE001
                # 추론 예외로 스레드가 죽으면 이후 라벨이 영영 안 나간다 → 잡아서 계속 돈다.
                # (모델 코드는 외부 리포지토리 소관이라 어떤 예외가 올지 모른다)
                self.get_logger().error(f'infer() 예외: {exc}', throttle_duration_sec=1.0)
                label = None
            else:
                # 기록은 infer() 바로 뒤에서 한다. 아래 렌더 인계·발행까지 넣으면
                # "추론 median" 이 추론이 아닌 값이 된다.
                # time.time() 이 아니라 monotonic: 시스템 시각이 뒤로 점프해도
                # 음수 소요시간이 나오지 않는다.
                self._metrics.record_inference((time.monotonic() - started) * 1000.0)

            # 예외가 났어도 넘긴다 — 마지막 상태라도 그려야 화면이 멈춘 것처럼 안 보인다.
            # (inference 객체 안에 저장된 pending 정보를 가져온다{swap})
            self._hand_off_render()

            if not self._command_pub.publish(label):
                break       # 종료 감지

    def _hand_off_render(self):
        """이번 프레임의 추론 결과를 렌더 워커에게 넘기고 깨운다. 워커 스레드 전용."""
        payload = self._inference.take_render_payload()
        if payload is None:
            return          # GUI 를 안 쓰는 구현체(기본 구현이 None 을 준다)
        with self._render_cv:
            # 낡은 payload 가 남아 있으면 그냥 덮어쓴다. 렌더가 밀렸다는 뜻이고,
            # 그 경우 그려야 할 것은 최신 것 하나뿐이다.
            self._render_payload = payload
            self._render_cv.notify()

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

        # 플래그만 세우면 렌더 워커는 wait_for 의 timeout 이 끝날 때까지 자고 있다.
        # 깨워 줘야 곧바로 종료 플래그를 보고 빠져나온다.
        with self._render_cv:
            self._render_cv.notify_all()

        # **_source.release() 보다 반드시 먼저 join 한다.** 이 워커는 read() 안에서
        # 블로킹돼 있을 수 있는데, 장치를 먼저 반납하면 해제된 장치를 읽게 된다.
        if self._worker_thread is not None:
            # 진행 중인 read()+infer() 한 바퀴(느려도 수백 ms)가 끝날 여유를 준다.
            self._worker_thread.join(timeout=self.WORKER_JOIN_TIMEOUT_SEC)
            if self._worker_thread.is_alive():
                # daemon=True 라서 프로세스 종료를 막지는 않는다. 알리기만 한다.
                self.get_logger().warn('캡처+추론 워커가 제때 끝나지 않아 그대로 종료합니다.')
            self._worker_thread = None

        if self._render_thread is not None:
            self._render_thread.join(timeout=self.WORKER_JOIN_TIMEOUT_SEC)
            if self._render_thread.is_alive():
                self.get_logger().warn('렌더 워커가 제때 끝나지 않아 그대로 종료합니다.')
            self._render_thread = None

        if self._inference is not None:
            # 렌더 워커를 join 한 **뒤에** 부른다. 그려는 중에 창을 닫으면 안 된다.
            self._inference.close()

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
