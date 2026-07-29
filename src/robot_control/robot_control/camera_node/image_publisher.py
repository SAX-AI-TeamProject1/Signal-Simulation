"""
책임 2 — 이미지 발행.

웹캠 프레임을 sensor_msgs/Image 로 만들어 image_webcam 토픽에 내보낸다.
"발행"은 다른 노드가 그 화면을 볼 수 있게 하는 것이다. 추론은 이 토픽이 아니라
프로세스 안의 큐로 프레임을 받으므로, 이걸 꺼도 추론과 주행은 정상 동작한다.
"""

from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from robot_control.camera_node.shutdown import is_shutting_down
from sensor_msgs.msg import Image


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

        (토픽 그래프에 항상 보이는 편이 디버깅에 낫다. ros2 topic list 에서
         토픽이 아예 사라지면 "노드가 죽었나"부터 의심하게 된다)

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
