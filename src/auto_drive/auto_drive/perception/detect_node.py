# 2026-08-02
# 시뮬 카메라에 잡힌 물체가 무엇인지 판별해서 박스를 그린 이미지로 내보내는 노드.
#
# 이 노드는 주행에 관여하지 않는다. 속도 명령을 만들지도, 발행하지도 않는다 —
# 오직 <ns>/detect_image 만 낸다. viz/ 의 뷰어 노드들과 같은 위치이고, marker_vision
# (같은 YOLO 를 쓰지만 cmd_vel_marker_slow 로 감속을 거는 노드)과는 목적이 다르다.
#
# marker_vision 을 고쳐 쓰지 않은 이유가 세 가지 있다:
#   1. marker_vision 은 'TrackMarker' 클래스만 찾는데 obstacle_detector.pt 에 그 클래스가
#      없다(있는 것은 PalletJack/Bucket/Cluttering/TrashCan/Person/Vehicle 여섯 개).
#      TrackMarker 검출기는 아직 학습된 적이 없다.
#   2. marker_vision 은 클래스 이름을 쓰지 않고 박스 높이만 본다 — 판별이 아니라 감속용이다.
#   3. marker_vision 은 import 가 깨져 있다(robot_control.patrol.track_marker_infer 를
#      가리키는데 그 파일은 패키지 분리 때 auto_drive.patrol 로 옮겨졌다). 그 노드는
#      기본이 꺼짐이라 아직 드러나지 않았을 뿐이다 — 여기서 같이 고치지는 않는다.
#
# "주행 성능에 영향을 주지 않는다"를 지키는 장치가 넷이다. 추론은 CPU 에서 프레임당
# 51ms(실측) 걸리는 무거운 작업이고, 이 노드는 물리 시뮬레이션과 같은 기계에서 도는
# 별도 프로세스라 CPU 를 나눠 쓰기 때문이다:
#   1. 구독 콜백은 프레임을 한 칸짜리 버퍼에 넣기만 하고 즉시 반환한다. 추론은 워커
#      스레드가 한다 — 콜백에서 추론하면 rclpy 실행기가 그동안 막혀 구독 큐가 밀린다.
#   2. 그 버퍼는 항상 최신 한 장만 들고 낡은 것은 버린다(signal_vision 의
#      LatestFrameBuffer 와 같은 정책). 추론이 카메라보다 느려도 지연이 쌓이지 않는다.
#   3. max_rate_hz 로 추론 주기 자체를 카메라 주기(15Hz)보다 낮게 묶는다. 기본 5Hz 면
#      코어 하나의 약 1/4 만 쓴다.
#   4. 아무도 <ns>/detect_image 를 구독하지 않으면 추론을 아예 건너뛴다. 보는 사람이
#      없을 때 CPU 를 쓰는 것은 순수한 낭비다(always_on 으로 끌 수 있다).

from pathlib import Path
import threading

from auto_drive.patrol.track_marker_infer import load_model, predict
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# 이 노드가 다룰 수 있는 인코딩. 시뮬 카메라(gz)는 rgb8 로 보낸다.
SUPPORTED_ENCODINGS = ('rgb8', 'bgr8')


def imgmsg_to_bgr(msg):
    """
    sensor_msgs/Image 를 YOLO 가 쓰는 BGR numpy 배열로 바꾼다.

    cv_bridge 를 쓰지 않는 이유: 이 기계의 cv_bridge 는 OpenCV 4.6 으로 빌드돼
    있는데 설치된 python cv2 는 5.0.0 이라 타입 상수가 어긋나 있다. 실측으로
    cv2_to_imgmsg(..., encoding='bgr8') 이 KeyError: 16 으로 죽고,
    encoding 을 생략하면 색상 포맷이 아닌 '8UC3' 이 붙어 rqt_image_view 가
    제대로 못 읽는다. 인코딩이 rgb8/bgr8 둘뿐이라 numpy 슬라이스로 충분하다.
    """
    if msg.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(
            f'지원하지 않는 인코딩: {msg.encoding!r} '
            f'(가능: {", ".join(SUPPORTED_ENCODINGS)})')
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    # rgb8 이면 채널을 뒤집어 BGR 로. ::-1 은 뷰라서 복사가 없다 — 뒤에서
    # ultralytics 가 자기 버퍼에 그리므로 원본을 건드릴 일도 없다.
    return frame[:, :, ::-1] if msg.encoding == 'rgb8' else frame


def bgr_to_imgmsg(frame, header):
    """BGR numpy 배열을 sensor_msgs/Image(bgr8)로 조립한다(위와 같은 이유로 수동)."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    # tobytes() 는 연속 메모리를 요구한다 — plot() 결과는 연속이지만,
    # 슬라이스 뷰가 들어와도 안전하도록 명시한다.
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


class LatestFrame:
    """
    구독 콜백 → 워커 스레드로 프레임을 넘기는 한 칸짜리 버퍼.

    정책과 이유는 signal_vision 의 LatestFrameBuffer 와 같다(항상 최신 한 장,
    낡은 것은 버림, 절대 블로킹 없음). 여기에 다시 쓴 것은 auto_drive 가
    signal_vision 을 의존하지 않기 때문이다 — 뷰어 노드 하나 때문에 패키지
    의존을 새로 만드는 것보다 이 15줄을 갖는 편이 싸다.
    """

    def __init__(self):
        """빈 버퍼를 만든다."""
        self._cond = threading.Condition()
        self._slot = None

    def put(self, frame):
        """프레임을 넣는다. 안에 있던 낡은 프레임은 덮어쓴다. 블로킹하지 않는다."""
        with self._cond:
            self._slot = frame
            self._cond.notify()

    def get(self, timeout):
        """프레임을 꺼낸다. timeout 초 안에 없으면 None(워커가 종료 플래그를 볼 기회)."""
        with self._cond:
            if self._slot is None:
                self._cond.wait(timeout)
            frame, self._slot = self._slot, None
            return frame


class DetectNode(Node):
    """시뮬 카메라 영상에서 물체를 판별해 박스를 그린 이미지로 재발행한다."""

    def __init__(self):
        """가중치를 읽고 추론 워커 스레드를 띄운다."""
        super().__init__('detect_node')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('output_topic', 'detect_image')
        self.declare_parameter('conf_threshold', 0.4)
        # 카메라는 15Hz 로 오지만 그 전부에 추론을 걸 이유가 없다. 사람이 보려고
        # 그리는 그림이라 5Hz 면 충분하고, 남는 CPU 는 물리 시뮬레이션 몫이다.
        self.declare_parameter('max_rate_hz', 5.0)
        # true 면 구독자가 없어도 계속 추론한다(로그만 보고 싶을 때).
        self.declare_parameter('always_on', False)
        # 화면을 거의 다 덮는 박스는 버린다. 이 모델은 창고 선반 벽을 마주 보면
        # 화면 전체(0,0)-(640,480)를 'Cluttering' 하나로 잡는다(실측 conf 0.97) —
        # 박스 변이 이미지 테두리와 겹쳐 눈에 보이지도 않고, "어디에 무엇이 있다"는
        # 정보도 0 이라 그리는 의미가 없다. 1.0 으로 두면 필터가 꺼진다.
        self.declare_parameter('max_area_ratio', 0.9)

        weights_path = self.get_parameter('weights_path').value
        if not weights_path or not Path(weights_path).is_file():
            # 가중치 없이 뜨면 아무것도 못 하면서 카메라만 구독한다 —
            # 그 상태를 조용히 유지하는 것보다 시작 시점에 원인을 내는 쪽이 낫다
            # (marker_vision 과 같은 패턴).
            raise RuntimeError(
                f'YOLO 가중치 파일을 찾을 수 없습니다: {weights_path!r}. '
                'auto_drive/models/obstacle_detector.pt 를 넘기세요 — 없으면 VS Code '
                '태스크 "6. PERCEPTION 패키지 설치" 로 받습니다.')

        self._conf = self.get_parameter('conf_threshold').value
        self._always_on = self.get_parameter('always_on').value
        self._max_area_ratio = self.get_parameter('max_area_ratio').value
        rate = max(0.1, self.get_parameter('max_rate_hz').value)
        self._min_interval_sec = 1.0 / rate

        self._model = load_model(weights_path)
        self._frames = LatestFrame()
        self._stop = threading.Event()
        self._last_infer_time = None

        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(Image, output_topic, 1)
        # depth 1 의 sensor 프로파일: 밀린 프레임을 큐에 쌓지 않고 최신만 받는다.
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._on_image, qos_profile_sensor_data)

        # daemon: 워커가 추론 중일 때 Ctrl-C 가 와도 프로세스가 매달리지 않게 한다.
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'판별 노드 시작 — 클래스 {sorted(self._model.names.values())}, '
            f'입력 {self.get_parameter("image_topic").value}, 출력 {output_topic}, '
            f'최대 {rate:g}Hz')

    def _on_image(self, msg):
        """프레임을 버퍼에 넣기만 하고 즉시 반환한다 — 추론은 워커가 한다."""
        self._frames.put(msg)

    def _should_infer(self):
        """지금 추론할 차례인지(구독자 유무 + 주기 제한) 판단한다."""
        if not self._always_on and self._pub.get_subscription_count() == 0:
            return False
        now = self.get_clock().now()
        if self._last_infer_time is not None:
            elapsed = (now - self._last_infer_time).nanoseconds * 1e-9
            if elapsed < self._min_interval_sec:
                return False
        self._last_infer_time = now
        return True

    def _run(self):
        """워커 스레드: 최신 프레임을 받아 추론하고 박스를 그려 발행한다."""
        while not self._stop.is_set():
            msg = self._frames.get(timeout=0.5)
            if msg is None or not self._should_infer():
                continue
            try:
                self._infer_and_publish(msg)
            except Exception as exc:      # noqa: BLE001 - 워커가 죽으면 노드가 조용히 멎는다
                self.get_logger().error(f'추론 실패: {exc}', throttle_duration_sec=5.0)

    def _drop_oversized(self, result, shape):
        """화면 대비 max_area_ratio 를 넘는 박스를 버린 Results 를 돌려준다."""
        if self._max_area_ratio >= 1.0 or not len(result.boxes):
            return result
        height, width = shape[:2]
        frame_area = float(width * height)
        keep = []
        for i, box in enumerate(result.boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            if (x2 - x1) * (y2 - y1) / frame_area <= self._max_area_ratio:
                keep.append(i)
        # Results 인덱싱은 박스 부분집합을 가진 새 Results 를 만든다 —
        # plot() 이 그대로 동작하므로 그리기 경로를 따로 손댈 필요가 없다.
        return result[keep]

    def _infer_and_publish(self, msg):
        """한 프레임을 추론해 라벨을 로그로 남기고 박스를 그린 이미지를 발행한다."""
        # BGR 로 바꾸는 이유: ultralytics 와 OpenCV 가 BGR 을 전제로 그린다.
        frame = imgmsg_to_bgr(msg)
        results = predict(self._model, frame, conf=self._conf)
        if not results:
            return
        result = self._drop_oversized(results[0], frame.shape)

        if len(result.boxes):
            names = result.names
            found = ', '.join(
                f'{names[int(b.cls[0])]} {float(b.conf[0]):.2f}' for b in result.boxes)
            self.get_logger().info(f'감지 {len(result.boxes)}개: {found}',
                                   throttle_duration_sec=1.0)

        # plot() 은 박스와 라벨을 그린 BGR 배열을 새로 만들어 돌려준다 — 직접
        # rectangle/putText 를 부르는 것보다 색·글꼴이 ultralytics 기본과 일관된다.
        annotated = result.plot()
        # 원본 프레임의 시각·frame_id 를 그대로 물려준다 — RViz 에서 tf 와 맞물려야 한다.
        self._pub.publish(bgr_to_imgmsg(annotated, msg.header))

    def destroy_node(self):
        """워커를 세우고 노드를 내린다."""
        self._stop.set()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        try:
            node = DetectNode()
        except RuntimeError as exc:
            rclpy.logging.get_logger('detect_node').fatal(str(exc))
            return
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT 는 KeyboardInterrupt, SIGTERM 은 ExternalShutdownException 으로 온다
        # (estop_node 와 같은 이유로 뒤엣것도 잡는다).
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
