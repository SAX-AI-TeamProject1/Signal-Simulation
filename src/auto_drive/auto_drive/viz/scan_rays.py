# 라이다 스캔을 점이 아니라 "선"으로 보기 위한 뷰어 전용 노드.
#
# RViz 의 LaserScan 디스플레이는 반사가 돌아온 자리에 점만 찍는다. 그래서 아무것도
# 맞히지 못한 광선(range 가 inf)은 화면에서 그냥 사라진다. 실측하면 전방 180도
# 스캔 360개 중 283개가 inf 였다(창고는 60x100m 인데 사거리는 10m) — 부채꼴의
# 79% 가 빈 화면이었다는 뜻이고, 그 상태로는 "센서가 저쪽을 안 본다"와 "봤는데
# 10m 안에 아무것도 없다"를 구분할 수 없다.
#
# 그래서 같은 스캔을 Marker LINE_LIST 두 개로 다시 그린다:
#   hit  — 반사가 온 광선. 라이다 원점에서 맞은 지점까지.
#   miss — inf 광선. 원점에서 사거리 끝까지, 흐린 색으로.
#
# /scan 자체는 절대 고치지 않는다. inf 를 사거리 값으로 바꿔 발행하면 estop 이나
# 나중의 Nav2 가 "10m 앞에 벽"으로 읽는다 — 보기 좋자고 주행 판단을 오염시킬 수는
# 없다. 그래서 별도 토픽으로만 내보내고, 이 노드가 죽어도 주행은 아무 영향이 없다.

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# 광선 색 기본값 (r, g, b, a). 처음에는 miss 를 alpha 0.22 로 아주 흐리게 뒀는데,
# RViz 배경이 어두운 회색(48,48,48)이라 화면에서 거의 안 보였다. 밝은 청록으로
# 바꾸고 alpha 를 올린다 — miss 선과 hit 선은 사실상 겹치지 않으므로(각 광선은
# 둘 중 하나에만 들어간다) 진하게 해도 hit 을 가리지 않는다.
HIT_COLOR = (1.0, 0.3, 0.2, 1.0)
MISS_COLOR = (0.3, 0.9, 1.0, 0.6)

# 스캔이 끊겼을 때 화면에 선이 남아 있지 않게 하는 수명. 10Hz 스캔의 3배 여유 —
# 한 프레임 걸러도 선이 유지되면서, 끊기면 0.3초 안에 사라진다.
MARKER_LIFETIME_SEC = 0.3


class ScanRays(Node):
    """LaserScan 을 광선 선분(Marker LINE_LIST)으로 다시 그려 발행한다."""

    def __init__(self):
        super().__init__('scan_rays')

        # 720개를 다 그리면 선 사이가 붙어 부채꼴이 통짜 면처럼 보인다. 몇 개 걸러
        # 그려야 "광선"으로 읽힌다 — 점(LaserScan 디스플레이)이 이미 전부 보여
        # 주므로 선 쪽은 성기어도 정보가 빠지지 않는다.
        self.declare_parameter('stride', 4)
        # 3cm 까지 올려 봤더니 선끼리 붙어 부채꼴이 뭉쳤다. 2cm 가 25m 궤도 뷰에서
        # 보이면서 선이 서로 안 겹치는 선이다.
        self.declare_parameter('line_width', 0.02)
        # 사거리 끝까지 그린 miss 선이 진짜 반사로 오해받지 않게 살짝 짧게 끊는다.
        self.declare_parameter('miss_scale', 0.95)
        # 색을 파라미터로 뺀 이유: 가시성은 화면을 보면서 맞춰야 하는 값이라,
        # 고치고 다시 띄우는 것보다 ros2 param set 으로 바로 바꾸는 게 빠르다.
        #   ros2 param set /robot2/scan_rays miss_color "[0.2, 0.9, 1.0, 0.8]"
        self.declare_parameter('hit_color', list(HIT_COLOR))
        self.declare_parameter('miss_color', list(MISS_COLOR))

        self.pub = self.create_publisher(MarkerArray, 'scan_rays', 1)
        # 스캔은 센서 데이터라 Best Effort 로 온다. Reliable 로 구독하면 아예 안 붙는다.
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        stride = max(1, self.get_parameter('stride').value)
        miss_scale = self.get_parameter('miss_scale').value

        hit_pts = []
        miss_pts = []
        for i in range(0, len(msg.ranges), stride):
            r = msg.ranges[i]
            angle = msg.angle_min + i * msg.angle_increment
            # isfinite() 는 "무한인가"가 아니라 "유한한가"다 — inf 도 nan 도 False.
            if math.isfinite(r) and msg.range_min <= r and r <= msg.range_max:
                self.append_ray(hit_pts, angle, r)
            else:
                # inf(사거리 안에 반사 없음)와 nan(무효)을 같이 묶는다. 둘 다
                # "이 방향으로 쐈는데 아무것도 못 받았다"는 같은 뜻으로 그린다.
                self.append_ray(miss_pts, angle, msg.range_max * miss_scale)

        hit_color = self.get_parameter('hit_color').value
        miss_color = self.get_parameter('miss_color').value
        out = MarkerArray()
        out.markers.append(self.build_marker(msg.header, 0, 'hit', hit_color, hit_pts))
        out.markers.append(self.build_marker(msg.header, 1, 'miss', miss_color, miss_pts))
        self.pub.publish(out)

    @staticmethod
    def append_ray(points, angle, length):
        """원점 → (angle, length) 선분 하나를 LINE_LIST 용으로 두 점씩 넣는다."""
        start = Point()
        end = Point()
        end.x = length * math.cos(angle)
        end.y = length * math.sin(angle)
        points.append(start)
        points.append(end)

    def build_marker(self, header, marker_id, ns, color, points):
        marker = Marker()
        # 프레임을 스캔 헤더 그대로 쓴다: 라이다 프레임 원점이 곧 광선의 출발점이라
        # 좌표 변환이 필요 없고, 로봇이 움직여도 선이 따라간다.
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.get_parameter('line_width').value
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = points
        marker.lifetime = rclpy.duration.Duration(
            seconds=MARKER_LIFETIME_SEC).to_msg()
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ScanRays()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
