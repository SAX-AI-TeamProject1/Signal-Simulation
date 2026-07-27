"""
웹캠(실물 카메라) → ROS 2 이미지 토픽 발행 노드.

역할은 프레임을 topic 으로 올리는 것뿐이다. 인식/추론은 하지 않는다
(핸드오프 §2 "만들지 말 것" — 추론은 gesture_ai_node 담당).

구독: 없음 (V4L2 장치를 직접 연다)
발행: image_webcam (sensor_msgs/Image, bgr8)

실행:
    ros2 run robot_control camera_node
    ros2 run robot_control camera_node --ros-args -p device_id:=1 -p fps:=15.0
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    """웹캠 프레임을 주기적으로 읽어 sensor_msgs/Image 로 발행한다."""

    def __init__(self):
        """파라미터를 선언하고 카메라 장치를 연 뒤 발행 타이머를 건다."""
        super().__init__('camera_node')

        # 파라미터로 빼둔 이유: launch 나 --ros-args -p 로 코드 수정 없이 바꾸려고.
        self.declare_parameter('device_id', 0)          # /dev/video<N> 의 N
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)             # 발행 주기(Hz)
        self.declare_parameter('frame_id', 'webcam')    # Image 헤더의 frame_id

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        self._frame_id = self.get_parameter('frame_id').value

        if fps <= 0.0:
            raise ValueError(f'fps 는 0 보다 커야 합니다 (받은 값: {fps})')

        # V4L2 를 명시적으로 지정: 지정하지 않으면 OpenCV 가 다른 백엔드를 잡아
        # 해상도/FPS 설정이 무시되는 경우가 있다.
        self._cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(
                f'웹캠을 열지 못했습니다: /dev/video{device_id} — '
                '장치 존재 여부(ls /dev/video*)와 권한(video 그룹)을 확인하세요.')
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        # 요청값과 실제값이 다를 수 있다(장치가 지원하는 모드로 스냅됨) → 실제값을 찍어둔다.
        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self._bridge = CvBridge()

        # QoS 는 qos_profile_sensor_data(BEST_EFFORT) 사용.
        # 주의: 구독 측(gesture_ai_node)도 같은 프로파일이어야 매칭된다.
        #       RELIABLE 구독자는 BEST_EFFORT 발행자와 매칭되지 않아 조용히 아무것도 못 받는다.
        self._pub = self.create_publisher(Image, 'image_webcam', qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / fps, self._on_timer)

        self.get_logger().info(
            f'웹캠 시작: /dev/video{device_id} {actual_w}x{actual_h} @{fps}Hz '
            f'→ {self._pub.topic_name}')

    def _on_timer(self):
        """프레임 한 장을 읽어 Image 메시지로 발행한다."""
        ok, frame = self._cap.read()
        if not ok:
            # 일시적 read 실패로 노드를 죽이지는 않는다. 로그만 억제해서 남긴다.
            self.get_logger().warn('프레임 읽기 실패 — 이번 주기는 건너뜁니다.',
                                   throttle_duration_sec=5.0)
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

    def destroy_node(self):
        """노드 종료 시 카메라 장치를 반드시 반납한다(안 하면 다음 실행에서 열리지 않음)."""
        if getattr(self, '_cap', None) is not None:
            self._cap.release()
            self._cap = None
        return super().destroy_node()


def main(args=None):
    """엔트리 포인트."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 나 launch 종료 신호. 정상 종료 경로이므로 traceback 을 남기지 않는다.
        pass
    except (RuntimeError, ValueError) as exc:
        # 카메라를 못 열었을 때 스택트레이스 대신 원인만 보여준다.
        rclpy.logging.get_logger('camera_node').fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
