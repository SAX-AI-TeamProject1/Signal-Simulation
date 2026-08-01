# map → <ns>/odom 을 시뮬레이터의 진짜 좌표(pose_gt)로 매번 보정해 발행한다.
#
# 왜 필요한가: RViz 에 그려지는 로봇 위치는 map → odom → base_link 를 타고 나오는데,
# 뒷단 odom → base_link 는 gz 의 DiffDrive 가 내는 바퀴 적산치다(urdf 의
# <tf_topic>$(arg ns)/tf</tf_topic> → bridge.yaml 이 /tf 로 넘긴다). 바퀴가 미끄러지면
# 이 값이 실제와 어긋나고 오차는 시간이 갈수록 쌓인다. 게다가 이 로봇은 메카넘인데
# 적산은 diff-drive 공식으로 하므로 옆으로 미끄러진 만큼은 아예 안 잡힌다.
# 그래서 map → odom 을 스폰 pose 로 고정해 두면, 로봇이 달릴수록 gz 창과 RViz 가
# 조금씩 벌어진다.
#
# 무엇을 하나: map → odom 을 고정값이 아니라 "진짜 위치에서 odom 을 되빼는 보정값"
# 으로 계산해 내보낸다.
#
#     map→odom = (map→base_link) x (odom→base_link)^-1
#
# 앞항이 pose_gt(진짜), 뒷항이 odom(적산치)이다. 두 오차가 상쇄되어 base_link 는
# 항상 진짜 자리에 놓인다. odom → base_link 자체는 건드리지 않는다 — 그건 gz 가
# 내는 센서값이고, 여기서 고쳐 쓰면 나중에 그 값으로 뭔가를 판단할 때 거짓이 된다.
#
# 이 자리는 원래 SLAM 이나 AMCL 이 맡는 자리다(그쪽도 map → odom 보정을 낸다).
# SLAM 이 들어오면 이 노드를 내리면 되고, 프레임 구성은 그대로 쓴다.
# 시뮬레이션에서만 성립한다 — 실장비에는 pose_gt 같은 게 없다.

from auto_drive.transforms import transform_inverse, transform_multiply
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def pose_to_transform(pose):
    """geometry_msgs/Pose 를 (translation, rotation) 튜플로."""
    p, q = pose.position, pose.orientation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class GroundTruthTf(Node):
    """pose_gt 와 odom 을 받아 map → <ns>/odom 보정 변환을 발행한다."""

    def __init__(self):
        super().__init__('ground_truth_tf')

        self.declare_parameter('map_frame', 'map')
        # 빈 값이면 이 노드의 네임스페이스에서 가져온다 — launch 가 로봇 ns 안에
        # 띄우므로 로봇마다 자기 프레임 이름이 자동으로 맞는다.
        self.declare_parameter('odom_frame', '')

        self.map_frame = self.get_parameter('map_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        if not odom_frame:
            ns = self.get_namespace().strip('/')
            odom_frame = f'{ns}/odom' if ns else 'odom'
        self.odom_frame = odom_frame

        self._truth = None
        self._broadcaster = TransformBroadcaster(self)

        # pose_gt 는 gz 의 PosePublisher 가 내는 값이라 header 가 없다(Pose 타입).
        # 그래서 시각은 odom 쪽 header 를 쓴다 — 둘 다 30Hz 로 같은 시계를 본다.
        self.create_subscription(Pose, 'pose_gt', self._on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.get_logger().info(f'{self.map_frame} → {self.odom_frame} 보정 시작')

    def _on_truth(self, msg):
        self._truth = pose_to_transform(msg)

    def _on_odom(self, msg):
        if self._truth is None:
            # 아직 진짜 좌표가 안 왔다. 여기서 항등 변환을 내보내면 로봇이 잠깐
            # 월드 원점에 붙었다가 튀므로, 올 때까지 아무것도 내보내지 않는다.
            return

        odom_to_base = pose_to_transform(msg.pose.pose)
        (tx, ty, tz), (qx, qy, qz, qw) = transform_multiply(
            self._truth, transform_inverse(*odom_to_base))

        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = tx
        out.transform.translation.y = ty
        out.transform.translation.z = tz
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self._broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT 는 KeyboardInterrupt, SIGTERM 은 ExternalShutdownException 으로 온다.
        # 뒤엣것을 안 잡으면 launch 가 노드를 내릴 때 역추적이 통째로 찍힌다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
