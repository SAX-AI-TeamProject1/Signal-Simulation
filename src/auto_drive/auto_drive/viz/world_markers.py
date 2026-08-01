# 월드(창고)의 형상을 RViz 로 옮겨 그리는 뷰어 전용 노드.
#
# RViz 화면에는 라이다 점과 선만 떠 있어서, "저 점이 왜 저기 찍혔는지"를 알 수 없었다.
# 벽인지 선반인지 카트인지는 gz sim 창을 따로 봐야만 구분됐다. gz 쪽 GUI 플러그인
# (Visualize Lidar)으로 광선을 창고 위에 겹쳐 보려 했지만 광선이 월드 원점(0,0,0)에만
# 그려졌다 — 그 플러그인은 토픽으로 센서 엔티티를 찾아 worldPose() 로 위치를 잡는데,
# 실행 중에 스폰된 이 로봇에서는 그 조회가 안 됐다. 그래서 반대로, 이미 점을 제 위치에
# 그리고 있는 RViz 쪽으로 월드 형상을 가져오기로 했다.
#
# RViz 의 리소스 로더는 model:// 를 모른다 (package://, file://, http:// 만 안다).
# 그래서 SDF 의 model://Foo/meshes/bar.dae 를 models_root 기준 절대경로로 풀어
# file:// 로 바꿔 넘긴다.
#
# 좌표계: 마커는 월드 좌표 그대로이므로 map 프레임에 실린다. RViz 의 fixed frame 이
# robot2/odom 이면 어긋나므로, launch 가 map → <ns>/odom 정적 변환을 함께 발행하고
# fixed frame 도 map 으로 옮긴다(doc/design.md 의 map 루트 결정).
#
# 이 노드는 주행에 관여하지 않는다. 죽어도 로봇은 그대로 달린다.

import os
import xml.etree.ElementTree as ET

from auto_drive.viz.sdf_geometry import (apply_up_axis_fix, compose, dae_header,
                                         MESH_COLOR, parse_pose, parse_scale,
                                         PRIMITIVE_COLOR, resolve_model_uri, to_ros_pose)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class WorldMarkers(Node):
    """월드 SDF 의 visual 을 읽어 MarkerArray 한 벌로 발행한다."""

    def __init__(self):
        super().__init__('world_markers')

        self.declare_parameter('world_path', '')
        self.declare_parameter('models_root', '')
        # 마커가 월드 좌표라서, 월드 원점을 나타내는 프레임 이름이 필요하다.
        self.declare_parameter('frame_id', 'map')
        # 바닥판(warehouse 의 바닥 메시)까지 그리면 스캔 점이 그 위에 묻혀 안 보인다.
        # 이름으로 걸러낼 수 있게 파라미터로 뺀다.
        self.declare_parameter('skip_models', [''])
        # COLLADA 보정 스위치 두 개. 어느 쪽이 맞는지는 화면을 봐야만 알 수 있어서
        # (gz 와 RViz 의 해석 차이는 assimp 버전에 따라 달라진다) 파라미터로 뺐고,
        # 값을 바꾸면 아래 콜백이 그 자리에서 다시 그린다 — 다시 띄울 필요가 없다.
        #   ros2 param set /world_markers apply_mesh_unit true
        #   ros2 param set /world_markers apply_up_axis false
        # 기본값의 근거는 dae_header() 주석에 있다.
        self.declare_parameter('apply_mesh_unit', False)
        self.declare_parameter('apply_up_axis', True)

        self.world_path = self.get_parameter('world_path').value
        self.models_root = self.get_parameter('models_root').value
        if not self.world_path or not os.path.isfile(self.world_path):
            raise RuntimeError(f'world_path 가 없다: {self.world_path!r}')
        if not self.models_root or not os.path.isdir(self.models_root):
            raise RuntimeError(f'models_root 가 없다: {self.models_root!r}')

        # transient_local: RViz 가 나중에 붙어도 마지막 발행분을 받는다. 한 번만
        # 쏘는 정적 데이터라 이게 없으면 순서 싸움에서 지면 화면이 비어 버린다.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(MarkerArray, 'world_markers', qos)
        self.publish_markers()
        self.add_post_set_parameters_callback(lambda _: self.publish_markers())

    def publish_markers(self):
        """월드를 다시 읽어 마커 한 벌을 만들고 발행한다."""
        self.apply_mesh_unit = self.get_parameter('apply_mesh_unit').value
        self.apply_up_axis = self.get_parameter('apply_up_axis').value
        self._unit_applied = 0
        self._up_axis_applied = 0
        markers = self.build_markers(self.world_path, self.models_root)
        self.pub.publish(markers)
        self.get_logger().info(
            f'월드 마커 {len(markers.markers)}개 발행 '
            f'(단위 보정 {self._unit_applied}개, Y_UP 보정 {self._up_axis_applied}개)')

    def build_markers(self, world_path, models_root):
        frame_id = self.get_parameter('frame_id').value
        skip = {n for n in self.get_parameter('skip_models').value if n}

        root = ET.parse(world_path).getroot()
        world = root.find('world')
        if world is None:
            raise RuntimeError(f'{world_path} 에 <world> 가 없다')

        out = MarkerArray()
        marker_id = 0
        skipped = 0
        for model in world.findall('model'):
            name = model.get('name', '')
            if name in skip:
                skipped += 1
                continue
            model_pose = parse_pose(model.find('pose'))
            for link in model.findall('link'):
                link_pose = compose(model_pose, parse_pose(link.find('pose')))
                for visual in link.findall('visual'):
                    pose = compose(link_pose, parse_pose(visual.find('pose')))
                    marker = self.build_marker(
                        visual, pose, frame_id, marker_id, models_root)
                    if marker is not None:
                        out.markers.append(marker)
                        marker_id += 1
        if skipped:
            self.get_logger().info(f'제외한 model {skipped}개: {sorted(skip)}')
        return out

    def build_marker(self, visual, pose, frame_id, marker_id, models_root):
        """SDF visual 하나를 Marker 로. 그릴 수 없는 형상이면 None 을 돌려준다."""
        geometry = visual.find('geometry')
        if geometry is None:
            return None

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = 'world'
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose = to_ros_pose(pose)

        mesh = geometry.find('mesh')
        box = geometry.find('box')
        cylinder = geometry.find('cylinder')
        sphere = geometry.find('sphere')

        if mesh is not None:
            path = self.resolve_uri(mesh.findtext('uri', ''), models_root)
            if path is None:
                return None
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = 'file://' + path
            # 메시 자체의 재질(텍스처)을 쓴다. 없으면 아래 색으로 떨어진다.
            marker.mesh_use_embedded_materials = True
            sx, sy, sz = parse_scale(mesh)
            unit, up_axis = dae_header(path)
            # SDF 의 scale 위에 파일이 선언한 단위를 한 번 더 곱한다(dae_header 주석).
            if self.apply_mesh_unit and unit != 1.0:
                sx, sy, sz = sx * unit, sy * unit, sz * unit
                self._unit_applied += 1
            if self.apply_up_axis and up_axis == 'Y_UP':
                apply_up_axis_fix(marker)
                self._up_axis_applied += 1
            color = MESH_COLOR
        elif box is not None:
            marker.type = Marker.CUBE
            sx, sy, sz = (float(v) for v in box.findtext('size', '1 1 1').split())
            color = PRIMITIVE_COLOR
        elif cylinder is not None:
            marker.type = Marker.CYLINDER
            radius = float(cylinder.findtext('radius', '0.5'))
            # CYLINDER 의 scale 은 지름 기준이라 반지름을 두 배로 넣는다.
            sx = sy = radius * 2.0
            sz = float(cylinder.findtext('length', '1.0'))
            color = PRIMITIVE_COLOR
        elif sphere is not None:
            marker.type = Marker.SPHERE
            sx = sy = sz = float(sphere.findtext('radius', '0.5')) * 2.0
            color = PRIMITIVE_COLOR
        else:
            # plane(무한 평면) 등은 마커로 대응되는 타입이 없어 건너뛴다.
            return None

        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def resolve_uri(self, uri, models_root):
        """메시 경로를 풀되, 못 풀면 경고만 남기고 None 을 돌려준다.

        여기서 예외를 올리지 않는 이유: 소품 하나가 빠질 뿐인데 노드가 죽으면
        나머지 200여 개도 같이 못 보게 된다.
        """
        path = resolve_model_uri(uri, models_root)
        if path is None:
            self.get_logger().warn(f'메시를 못 찾았다: {uri!r}')
        return path


def main(args=None):
    rclpy.init(args=args)
    node = WorldMarkers()
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
