#!/usr/bin/env python3
"""
aws-robomaker-small-warehouse-world (Gazebo Classic, SDF 1.6)
  -> Gazebo Harmonic(gz-sim) / ROS 2 Jazzy 용 변환 스크립트

사용법:
    python3 port_aws_warehouse_to_gz.py <repo_root>

수행 작업 (모두 원본 파일 검사로 확인된 비호환 항목):
 1) models/*/model.sdf 의 메시 URI
      file://models/<M>/meshes/x.DAE  ->  model://<M>/meshes/x.DAE
    (Classic 은 GAZEBO_RESOURCE_PATH 기준 상대경로를 허용했지만,
     gz-sim 은 model:// + GZ_SIM_RESOURCE_PATH 가 표준)
 2) worlds/*.world 의
      <model name="A"><include><uri>..</uri></include><pose/></model>  (중첩 모델)
      ->  <include><uri>..</uri><name>A</name><pose>..</pose></include>  (월드 직속 include)
 3) <pose frame="">  ->  <pose>          (frame 속성은 SDF 1.7 에서 제거됨)
 4) Classic 전용 <gui><camera name="user_camera"> 블록 삭제
 5) gz-sim 시스템 플러그인(Physics/UserCommands/SceneBroadcaster/Sensors/Imu/Contact)과
    directional light(sun) 추가  — 원본 월드에는 point light 1개뿐이라 매우 어두움

원본은 <file>.bak 으로 백업.
XML 주석 안에 들어있던 모델(RoofB_01_001, DeskC_01_001~003)은 애초에 비활성 상태라 삭제됨.
"""
import sys, pathlib, shutil, re
import xml.etree.ElementTree as ET

SYSTEMS_XML = """
<root>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.5 0.1 -0.9</direction>
  </light>
</root>
"""

def port_models(root: pathlib.Path) -> int:
    n = 0
    for sdf in sorted((root / "models").glob("*/model.sdf")):
        txt = sdf.read_text()
        new = re.sub(r"file://models/([A-Za-z0-9_]+)/", r"model://\1/", txt)
        if new != txt:
            shutil.copy(sdf, str(sdf) + ".bak")
            sdf.write_text(new)
            n += 1
    return n

def port_world(path: pathlib.Path) -> int:
    tree = ET.parse(path)            # ET 는 주석을 자동으로 버림
    sdf = tree.getroot()
    world = sdf.find("world")
    flattened = 0

    for model in list(world.findall("model")):
        inc = model.find("include")
        if inc is None or model.find("link") is not None:
            continue                  # 실제 링크를 가진 모델은 건드리지 않음
        uri  = inc.findtext("uri")
        pose = (model.findtext("pose") or "0 0 0 0 0 0").strip()
        name = model.get("name")
        idx  = list(world).index(model)
        world.remove(model)

        new_inc = ET.Element("include")
        ET.SubElement(new_inc, "uri").text  = uri
        ET.SubElement(new_inc, "name").text = name
        ET.SubElement(new_inc, "pose").text = pose
        world.insert(idx, new_inc)
        flattened += 1

    for gui in world.findall("gui"):  # Classic user_camera 문법
        world.remove(gui)

    for el in world.iter():           # SDF 1.7+ 에서 제거된 frame 속성
        el.attrib.pop("frame", None)

    extra = ET.fromstring(SYSTEMS_XML)
    phys = world.find("physics")
    at = list(world).index(phys) + 1 if phys is not None else 0
    for i, child in enumerate(list(extra)):
        world.insert(at + i, child)

    shutil.copy(path, str(path) + ".bak")
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return flattened

if __name__ == "__main__":
    root = pathlib.Path(sys.argv[1]).resolve()
    print(f"[models] model.sdf {port_models(root)}개 URI 수정")
    for w in sorted(root.glob("worlds/*/*.world")):
        print(f"[world ] {w.name}: include {port_world(w)}개 평탄화")

'''
cd ~/projects/myproj
source .venv/bin/activate   # 직접 해야 함
python train.py
'''