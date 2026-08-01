# 2026-07-30
#
# waypoint_follower(pose_gt 기반)는 코너에 "도착"해야만 방향을 트는데, 제어 주기(1.2s)
# 동안 순항 속도(5m/s)로 최대 6m를 더 갈 수 있어 코너를 지나친 뒤에야 급하게 꺾여
# 전복/이탈이 나던 문제가 있었다. 이 노드는 그 문제를 waypoint_follower를 갈아엎지
# 않고 "코너 표지(TrackMarker)가 카메라에 크게 잡히면 미리 감속"하는 방식으로 보강한다.
#
# 설계: 이 노드는 방향을 계산하지 않는다 — waypoint_follower가 내는 cmd_vel_auto를
# 그대로 받아 속도만 줄여서 더 높은 우선순위 토픽(cmd_vel_marker_slow)으로 재발행한다.
# twist_mux가 이 토픽을 auto보다 우선(집행)하게 설정돼 있어(config/twist_mux.yaml),
# 마커가 안 보이면 이 노드는 그냥 발행을 멈추고 twist_mux의 timeout이 지나면 자동으로
# auto가 다시 통과된다 — 별도의 "해제" 로직이 필요 없다.
#
# TrackMarker 검출 모델(Signal-transport-perception에서 재학습)이 아직 없다 — 학습
# 파이프라인(capture_gazebo_dataset.py의 PRIMITIVE_TARGETS["TrackMarker"])은 준비돼
# 있지만 실제 캡처+학습(.venv-gz, Kaggle 인증 필요)은 아직 실행 전이다. weights_path가
# 없으면 생성자에서 바로 에러를 내 원인을 분명히 한다(카메라 못 여는 경우와 동일 패턴).

from pathlib import Path

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from robot_control.patrol.track_marker_infer import load_model, predict
from sensor_msgs.msg import Image

TRACK_MARKER_CLASS = 'TrackMarker'


class MarkerVision(Node):
    def __init__(self):
        super().__init__('marker_vision')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('cmd_vel_auto_topic', 'cmd_vel_auto')
        self.declare_parameter('output_topic', 'cmd_vel_marker_slow')
        self.declare_parameter('conf_threshold', 0.4)
        # 마커 바운딩박스 높이/이미지 높이 비율. near_start부터 서서히 감속을 시작해서
        # near_full에서 min_speed_factor까지 떨어진다(waypoint_follower의 speed_factor와
        # 같은 선형 보간 방식 — 급감속으로 인한 전복을 피하려는 이유도 동일).
        self.declare_parameter('near_start_ratio', 0.12)
        self.declare_parameter('near_full_ratio', 0.35)
        self.declare_parameter('min_speed_factor', 0.2)

        weights_path = self.get_parameter('weights_path').value
        if not weights_path or not Path(weights_path).is_file():
            raise RuntimeError(
                f"TrackMarker 가중치 파일을 찾을 수 없습니다: '{weights_path}'. "
                'Signal-transport-perception에서 capture_gazebo_dataset.py TrackMarker '
                '→ prepare_dataset.py → train_kaggle.sh 로 먼저 학습해야 합니다. '
                '당장 필요 없으면 enable_marker_vision:=false 로 끄세요.')

        self._image_topic = self.get_parameter('image_topic').value
        self._conf_threshold = self.get_parameter('conf_threshold').value
        self._near_start = self.get_parameter('near_start_ratio').value
        self._near_full = self.get_parameter('near_full_ratio').value
        self._min_factor = self.get_parameter('min_speed_factor').value

        self._bridge = CvBridge()
        self._model = load_model(weights_path)
        self._latest_cmd = None

        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_auto_topic').value, self._on_cmd_auto, 10)
        self.create_subscription(Image, self._image_topic, self._on_image, 10)
        self._pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        self.get_logger().info(
            f'marker_vision 시작 — weights={weights_path}, image_topic={self._image_topic}')

    def _on_cmd_auto(self, msg: Twist) -> None:
        self._latest_cmd = msg

    def _speed_factor(self, bbox_height_ratio: float) -> float | None:
        """근접도에 따라 0~1 사이로 부드럽게 줄어드는 배율. 너무 멀면 None(감속 불필요)."""
        if bbox_height_ratio <= self._near_start:
            return None
        if bbox_height_ratio >= self._near_full:
            return self._min_factor
        span = self._near_full - self._near_start
        t = (bbox_height_ratio - self._near_start) / span
        return 1.0 - t * (1.0 - self._min_factor)

    def _on_image(self, msg: Image) -> None:
        if self._latest_cmd is None:
            return  # waypoint_follower가 아직 아무것도 발행 안 함 — 줄일 대상이 없음

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = predict(self._model, frame, conf=self._conf_threshold)
        if not results:
            return

        result = results[0]
        names = result.names
        best_height_ratio = 0.0
        img_h = frame.shape[0]
        for box in result.boxes:
            class_id = int(box.cls[0])
            if names.get(class_id) != TRACK_MARKER_CLASS:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            ratio = (y2 - y1) / img_h
            best_height_ratio = max(best_height_ratio, ratio)

        factor = self._speed_factor(best_height_ratio)
        if factor is None:
            return  # 마커가 안 보이거나 아직 멀다 — 발행 안 함, twist_mux가 auto로 되돌아감

        slowed = Twist()
        slowed.linear.x = self._latest_cmd.linear.x * factor
        slowed.linear.y = self._latest_cmd.linear.y * factor
        slowed.angular.z = self._latest_cmd.angular.z
        self._pub.publish(slowed)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        try:
            node = MarkerVision()
        except RuntimeError as exc:
            rclpy.logging.get_logger('marker_vision').fatal(str(exc))
            return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
