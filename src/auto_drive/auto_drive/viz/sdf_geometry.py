# SDF 의 형상 정보를 RViz 마커로 옮길 때 쓰는 공용 부분.
#
# world_markers(창고 소품)와 actor_markers(돌아다니는 사람)가 같은 일을 한다:
# SDF 의 pose 를 읽어 합성하고, model:// 메시 경로를 RViz 가 읽을 수 있는 file:// 로
# 풀고, gz 와 RViz 의 COLLADA 해석 차이를 메운다. 두 노드에 같은 코드를 두면 한쪽만
# 고쳐져 어긋나므로 여기 모은다.

import math
import os
import re
import xml.etree.ElementTree as ET  # noqa: F401  (사용하는 쪽의 타입 힌트용)

from auto_drive.transforms import quaternion_from_rpy, quaternion_multiply
from geometry_msgs.msg import Pose

# 메시에 재질이 들어 있으면 그걸 쓰고, 없으면(STL 등) 이 색으로 그린다. 흰색인 이유는
# 재질이 있는 메시에 색을 곱해도 원래 색이 그대로 남기 때문 — 어둡게 물들지 않는다.
MESH_COLOR = (1.0, 1.0, 1.0, 1.0)
# box/cylinder 는 창고 구조물(벽·기둥)이 대부분이라 메시와 구분되게 살짝 푸른 회색.
PRIMITIVE_COLOR = (0.55, 0.58, 0.65, 1.0)

# Y_UP COLLADA 를 Z-up 세계에 세우는 보정: X축 +90도. (x,y,z)→(x,-z,y) 가 된다.
Y_UP_FIX = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))

# COLLADA 헤더의 <unit meter="0.01"/> 와 <up_axis> 를 뽑는다. 둘 다 파일 맨 앞
# <asset> 안에만 나오므로 통째로 읽지 않는다 — 창고 메시 하나가 300KB 다.
_UNIT_RE = re.compile(rb'<unit[^>]*meter="([0-9.eE+-]+)"')
_UP_AXIS_RE = re.compile(rb'<up_axis>\s*([A-Z_]+)\s*</up_axis>')
_DAE_SUFFIXES = ('.dae', '.DAE')
_header_cache = {}

MODEL_SCHEME = 'model://'


def parse_pose(element):
    """<pose>x y z r p y</pose> 를 (x,y,z,r,p,y) 로. 없거나 짧으면 0 으로 채운다."""
    if element is None or not (element.text or '').strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return parse_pose_text(element.text)


def parse_pose_text(text):
    """공백으로 나뉜 pose 문자열을 (x,y,z,r,p,y) 로."""
    parts = [float(v) for v in (text or '').split()]
    parts += [0.0] * (6 - len(parts))
    return tuple(parts[:6])


def compose(parent, child):
    """부모 pose 위에 자식 pose 를 얹는다. 위치는 부모 yaw 로 돌리고 각도는 더한다.

    일반적인 3축 회전 합성이 아니라 이 형태로 충분한 근거: 이 월드의 pose 782개를
    세어 보니 roll 또는 pitch 가 0 이 아닌 것은 4개뿐이고 전부 말단 visual 이었다
    (mecanum_lift_* 3개와 hand_signal_person). 부모가 되는 model/link pose 는 전부
    yaw 전용이고, 부모 회전이 R_z 뿐이면 R_z(p)·R_z(c)·R_x(r) 이 그대로 (roll=r,
    pitch=0, yaw=p+c) 로 떨어져 이 식이 정확한 답이 된다.

    단, 앞으로 model 이나 link pose 에 roll/pitch 가 붙으면 그때부터는 근사가 된다.
    그런 소품이 들어오면 여기를 제대로 된 쿼터니언 합성으로 바꿔야 한다.
    """
    px, py, pz, pr, pp, pyaw = parent
    cx, cy, cz, cr, cp, cyaw = child
    cos_y, sin_y = math.cos(pyaw), math.sin(pyaw)
    return (
        px + cx * cos_y - cy * sin_y,
        py + cx * sin_y + cy * cos_y,
        pz + cz,
        pr + cr,
        pp + cp,
        pyaw + cyaw,
    )


def to_ros_pose(pose6):
    """(x,y,z,r,p,y) 를 geometry_msgs/Pose 로."""
    x, y, z, roll, pitch, yaw = pose6
    out = Pose()
    out.position.x, out.position.y, out.position.z = x, y, z
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    out.orientation.x, out.orientation.y = qx, qy
    out.orientation.z, out.orientation.w = qz, qw
    return out


def dae_header(path):
    """COLLADA 헤더에서 (단위 환산 계수, up_axis) 를 읽는다. DAE 가 아니면 (1.0, None).

    gz 와 RViz 가 같은 파일을 다르게 해석할 때 그 차이를 메우려고 읽는다.

    단위: RViz(assimp)는 <unit> 을 이미 적용한다. 화면으로 확인했다 — 여기서 한 번
    더 곱했더니 unit=0.01 인 AWS 선반 16개가 100배 작아져 화면에서 사라졌고,
    unit=1.0 인 shelf_big_movai 같은 메시만 남았다. 그래서 apply_mesh_unit 은 기본
    꺼짐이다. 파일이 선언하는 값 자체는 실재한다(AWS 선반은 정점 범위 391.7 x 88.0
    x 261.3 에 unit 0.01 이라 실물 3.92 x 0.88 x 2.61 m 다). 다른 RViz/assimp
    조합에서 크기가 100배로 어긋나면 그때 켜면 된다.

    축: gz 는 Y_UP 을 Z-up 으로 돌려 세우는데, RViz 는 assimp 에
    IMPORT_COLLADA_IGNORE_UP_DIRECTION 을 켜 두어 돌리지 않는다. 그래서 Y_UP 메시만
    X축으로 90도 누워 보인다. 이 월드에서 Y_UP 은 MecanumLift.dae(3회)와
    coke_can.dae(12회) 둘뿐이고 나머지 24종은 Z_UP 이다.
    """
    if not path.endswith(_DAE_SUFFIXES):
        return (1.0, None)
    if path in _header_cache:
        return _header_cache[path]
    unit, up_axis = 1.0, None
    try:
        with open(path, 'rb') as handle:
            head = handle.read(4096)
        found = _UNIT_RE.search(head)
        if found:
            unit = float(found.group(1))
        found = _UP_AXIS_RE.search(head)
        if found:
            up_axis = found.group(1).decode()
    except (OSError, ValueError):
        unit, up_axis = 1.0, None
    _header_cache[path] = (unit, up_axis)
    return (unit, up_axis)


def parse_scale(mesh_element):
    """<mesh><scale> 을 (sx, sy, sz) 로. 없으면 1 배."""
    text = mesh_element.findtext('scale')
    if not text or not text.strip():
        return (1.0, 1.0, 1.0)
    parts = [float(v) for v in text.split()]
    parts += [parts[-1]] * (3 - len(parts))
    return tuple(parts[:3])


def resolve_model_uri(uri, models_root):
    """model://Foo/meshes/bar.dae 를 models_root 아래 절대경로로 푼다.

    RViz 의 resource_retriever 는 package://, file://, http:// 만 안다 — model:// 은
    gz 의 표기라 그대로 넘기면 아무것도 안 그려진다.
    """
    uri = (uri or '').strip()
    if not uri.startswith(MODEL_SCHEME):
        return None
    path = os.path.join(models_root, uri[len(MODEL_SCHEME):])
    return path if os.path.isfile(path) else None


def apply_up_axis_fix(marker):
    """마커 자세를 로컬 X축으로 90도 더 돌려 Y_UP 메시를 세운다.

    SDF 회전 "뒤에" 곱해야 한다 — 메시 자체의 축을 고치는 것이지 배치를 바꾸는 게
    아니라서, 앞에 곱하면 배치 회전까지 같이 돌아간다.
    """
    q = (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w)
    qx, qy, qz, qw = quaternion_multiply(q, Y_UP_FIX)
    marker.pose.orientation.x, marker.pose.orientation.y = qx, qy
    marker.pose.orientation.z, marker.pose.orientation.w = qz, qw
