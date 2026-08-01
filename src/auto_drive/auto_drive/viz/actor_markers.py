# 월드를 돌아다니는 사람(SDF <actor>)을 RViz 마커로 그리는 뷰어 전용 노드.
#
# world_markers 는 소품을 한 번만 그리고 끝낸다 — 안 움직이니까. actor 는 움직이므로
# 같은 방식으로는 못 그린다. 한 번 그려 두면 사람이 출발 지점에 박힌 채로 남는다.
#
# 위치를 어디서 얻나: gz 의 pose 토픽을 새로 브리지하는 대신, SDF 에 적힌 궤적을
# 시뮬레이션 시각으로 직접 풀어 쓴다. 이 월드의 actor 8명은 전부 다음 형태다.
#   - 웨이포인트 3개 (A → B → A), <loop>true</loop>, <auto_start>true</auto_start>
#   - 세 웨이포인트의 yaw 가 서로 같다 (회전 보간이 필요 없다)
# 즉 주기가 고정된 왕복 직선이라, 시각만 알면 위치가 정해진다. 브리지 항목을 늘리지
# 않아도 되고 /clock 은 이미 브리지되어 있다.
#
# 대신 이건 gz 의 애니메이션을 다시 구현한 것이라 원본과 어긋날 수 있다. 보간 방식이
# 다르면(예: gz 가 선형이 아닌 다른 곡선을 쓰면) 위치가 조금 다르게 나온다. 웨이포인트
# 위와 왕복 주기는 정확히 맞지만, 그 사이 구간은 "선형"이라는 가정이 들어간다.
#
# 골격 애니메이션은 재현하지 않는다. RViz 의 마커는 스키닝을 못 하므로 사람은 걷는
# 자세 그대로 미끄러진다. 위치를 보는 게 목적이라 그대로 둔다.

import math
import xml.etree.ElementTree as ET

from auto_drive.viz.sdf_geometry import (apply_up_axis_fix, dae_header, MESH_COLOR,
                                         parse_pose_text, parse_scale, resolve_model_uri,
                                         to_ros_pose)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# 마커가 남지 않게 하는 수명. 발행 주기보다 넉넉히 길게 잡되, 노드가 죽으면 사람이
# 화면에 박힌 채 남지 않을 만큼 짧게.
MARKER_LIFETIME_SEC = 1.0


def shortest_angle(start, end):
    """두 각도 사이의 최단 차이를 (-pi, pi] 로 돌려준다."""
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


class ActorTrack:
    """actor 하나의 메시와 궤적. 시각을 주면 그때의 pose 를 돌려준다."""

    def __init__(self, name, mesh_path, scale, waypoints, loop, delay_start):
        self.name = name
        self.mesh_path = mesh_path
        self.scale = scale
        self.waypoints = waypoints          # [(time, pose6), ...] 시간 오름차순
        self.loop = loop
        self.delay_start = delay_start
        self.period = waypoints[-1][0]

    def pose_at(self, sim_time):
        local = sim_time - self.delay_start
        if local <= 0.0:
            return self.waypoints[0][1]
        if self.period <= 0.0:
            return self.waypoints[0][1]
        if self.loop:
            local %= self.period
        elif local >= self.period:
            return self.waypoints[-1][1]

        for index in range(len(self.waypoints) - 1):
            t0, p0 = self.waypoints[index]
            t1, p1 = self.waypoints[index + 1]
            if t0 <= local <= t1:
                span = t1 - t0
                alpha = 0.0 if span <= 0.0 else (local - t0) / span
                return self.interpolate(p0, p1, alpha)
        return self.waypoints[-1][1]

    @staticmethod
    def interpolate(p0, p1, alpha):
        """위치는 선형, 각도는 최단 경로로 보간한다."""
        out = []
        for index in range(3):
            out.append(p0[index] + (p1[index] - p0[index]) * alpha)
        for index in range(3, 6):
            out.append(p0[index] + shortest_angle(p0[index], p1[index]) * alpha)
        return tuple(out)


class ActorMarkers(Node):
    """SDF 의 <actor> 궤적을 풀어 사람 마커를 주기적으로 발행한다."""

    def __init__(self):
        super().__init__('actor_markers')

        self.declare_parameter('world_path', '')
        self.declare_parameter('models_root', '')
        self.declare_parameter('frame_id', 'map')
        # 20Hz. gz 쪽 actor 갱신을 눈으로 따라가기에 충분하고, 사람이 8명뿐이라
        # 마커 8개를 다시 만드는 비용은 무시할 만하다.
        self.declare_parameter('publish_rate', 20.0)

        world_path = self.get_parameter('world_path').value
        models_root = self.get_parameter('models_root').value
        if not world_path:
            raise RuntimeError(f'world_path 가 없다: {world_path!r}')
        if not models_root:
            raise RuntimeError(f'models_root 가 없다: {models_root!r}')

        self.frame_id = self.get_parameter('frame_id').value
        self.actors = self.load_actors(world_path, models_root)

        self.pub = self.create_publisher(MarkerArray, 'world_actors', 1)
        rate = max(1.0, self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(f'actor {len(self.actors)}명 궤적 로드')

    def load_actors(self, world_path, models_root):
        root = ET.parse(world_path).getroot()
        world = root.find('world')
        if world is None:
            raise RuntimeError(f'{world_path} 에 <world> 가 없다')

        tracks = []
        for actor in world.findall('actor'):
            name = actor.get('name', 'actor')
            mesh_path, scale = self.find_mesh(actor, models_root)
            if mesh_path is None:
                self.get_logger().warn(f'{name}: 그릴 메시가 없어 건너뛴다')
                continue
            script = actor.find('script')
            if script is None:
                self.get_logger().warn(f'{name}: <script> 가 없어 건너뛴다')
                continue
            trajectory = script.find('trajectory')
            if trajectory is None:
                self.get_logger().warn(f'{name}: <trajectory> 가 없어 건너뛴다')
                continue

            waypoints = []
            for point in trajectory.findall('waypoint'):
                waypoints.append((float(point.findtext('time', '0')),
                                  parse_pose_text(point.findtext('pose', ''))))
            if not waypoints:
                self.get_logger().warn(f'{name}: 웨이포인트가 없어 건너뛴다')
                continue
            waypoints.sort(key=lambda item: item[0])

            tracks.append(ActorTrack(
                name=name,
                mesh_path=mesh_path,
                scale=scale,
                waypoints=waypoints,
                loop=(script.findtext('loop', 'true').strip().lower() == 'true'),
                delay_start=float(script.findtext('delay_start', '0') or 0.0),
            ))
        return tracks

    def find_mesh(self, actor, models_root):
        """이 actor 를 그릴 메시를 고른다: link 의 visual 이 먼저, 없으면 skin.

        이 월드의 actor 8명 중 7명은 <skin> 이 없고 <link><visual> 로 그려진다.
        gz 도 같은 순서로 떨어진다 — skin 이 없으면 링크 visual 을 렌더한다
        (시작할 때 뜨는 __default__ 메시 에러가 그 흔적이다, doc/design.md).
        """
        for link in actor.findall('link'):
            for visual in link.findall('visual'):
                geometry = visual.find('geometry')
                mesh = geometry.find('mesh') if geometry is not None else None
                if mesh is None:
                    continue
                path = resolve_model_uri(mesh.findtext('uri', ''), models_root)
                if path is not None:
                    return path, parse_scale(mesh)
        skin = actor.find('skin')
        if skin is not None:
            path = resolve_model_uri(skin.findtext('filename', ''), models_root)
            if path is not None:
                value = float(skin.findtext('scale', '1') or 1.0)
                return path, (value, value, value)
        return None, None

    def _on_timer(self):
        now = self.get_clock().now()
        sim_time = now.nanoseconds * 1e-9

        out = MarkerArray()
        for index, actor in enumerate(self.actors):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now.to_msg()
            marker.ns = 'actors'
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = 'file://' + actor.mesh_path
            marker.mesh_use_embedded_materials = True
            marker.pose = to_ros_pose(actor.pose_at(sim_time))
            _, up_axis = dae_header(actor.mesh_path)
            if up_axis == 'Y_UP':
                apply_up_axis_fix(marker)
            marker.scale.x, marker.scale.y, marker.scale.z = actor.scale
            (marker.color.r, marker.color.g,
             marker.color.b, marker.color.a) = MESH_COLOR
            marker.lifetime = rclpy.duration.Duration(
                seconds=MARKER_LIFETIME_SEC).to_msg()
            out.markers.append(marker)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ActorMarkers()
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
