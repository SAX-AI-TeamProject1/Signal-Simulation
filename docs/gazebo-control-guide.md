# Moving Transport Equipment on Tracks — Setup Guide
<!-- 트랙을 따라 움직이는 운송장비 — 구성 가이드 -->

**Note on the Korean lines in this file:** they are HTML comments (`<!-- -->`) placed only for Korean-speaking human readers. They are not additional instructions — do not parse, translate, or derive behavior from them. Treat the English text as the sole operative content.

This English-body + Korean-comment pattern applies to any bilingual doc in this repo, not just this file (e.g. `.claude/local/*.md`).
Before finalizing a bilingual doc, re-read the diff and verify two things:
- Every operative English line has its paired Korean comment — including lines synthesized fresh from conversation, not only ones translated from pre-existing Korean text.
  <!-- 모든 영어 본문 줄에 짝이 되는 한글 주석이 있는지 — 기존 한글을 번역한 줄뿐 아니라, 대화에서 새로 종합해 쓴 줄도 포함해서. -->
- No Korean word has leaked into the English body itself (e.g. echoing a user's own Korean phrasing verbatim into what should be an English line).
  <!-- 영어 본문 안에 한글 단어가 섞여 들어가지 않았는지 (예: 사용자가 쓴 한글 표현을 영어여야 할 줄에 그대로 옮겨 적는 경우). -->

A new bilingual doc needs its own copy of this whole note at the top — this note is self-referential to this file and does not automatically extend to other files.

## 1. Goal
<!-- 1. 목표 -->

- N transport vehicles patrol the warehouse, each repeating its track forever.
  <!-- 운송장비 N대가 창고 안에서 각자의 트랙을 무한 반복하며 순찰한다. -->
- Exactly one approach track leads to the signal point — the spot where a vehicle gathers to receive hand-signal commands (stop / turn left / turn right) from the signal worker.
  <!-- 수신호 포인트(신호수의 정지/좌회전/우회전 수신호를 받는 집결 지점)로 이어지는 트랙은 정확히 1개다. -->
- Any vehicle that carries a LiDAR must be a real physics model; decoration-only vehicles can be lightweight actors (see §2).
  <!-- 라이다를 다는 장비는 반드시 실제 물리 모델이어야 하고, 배경 장식용 장비는 가벼운 actor로 충분하다 (2장 참고). -->

## 2. Two ways to keep an object moving — pick per vehicle
<!-- 2. 객체를 계속 움직이게 하는 두 가지 방법 — 장비별로 선택 -->

**Method A — `<actor>` with a scripted trajectory.** Gazebo interpolates the model through timed waypoints and loops forever. No physics, near-zero CPU cost, ideal for our slow QEMU VM.
<!-- 방법 A — 스크립트 궤적을 가진 <actor>. 시간이 지정된 웨이포인트 사이를 보간하며 무한 반복한다. 물리 연산이 없어 CPU 부담이 거의 없고, 느린 QEMU VM에 최적. -->

**Method B — a real model with the `DiffDrive` plugin, driven by a waypoint-follower ROS 2 node publishing `cmd_vel`.** Full physics and collisions, and the only option that supports mounted sensors.
<!-- 방법 B — DiffDrive 플러그인을 단 실제 모델을, 웨이포인트를 순회하며 cmd_vel을 발행하는 ROS 2 노드로 구동. 물리·충돌이 실제로 동작하며, 센서 장착이 가능한 유일한 방법. -->

Decision rules:
<!-- 선택 기준: -->

- An `<actor>` cannot carry sensors — a LiDAR attached to an actor will not publish anything. LiDAR-carrying vehicles must use Method B.
  <!-- actor에는 센서를 달 수 없다 — actor에 붙인 라이다는 아무것도 발행하지 않는다. 라이다 장착 장비는 반드시 방법 B를 쓸 것. -->
- Actors have no collision body (other objects pass through them), but they ARE visible to cameras and `gpu_lidar`, so Method A vehicles still work as moving obstacles for perception tests.
  <!-- actor는 충돌체가 없어 다른 물체가 그냥 통과하지만, 카메라와 gpu_lidar에는 보인다. 따라서 방법 A 장비도 인지 테스트용 '움직이는 장애물' 역할은 할 수 있다. -->
- Start with 1 vehicle on Method B (the one that approaches the signal point, with LiDAR) and make the remaining N−1 patrol vehicles Method A actors. Promote actors to Method B only when needed.
  <!-- 우선 방법 B 1대(라이다를 달고 수신호 포인트로 접근하는 장비)로 시작하고, 나머지 순찰 장비 N−1대는 방법 A actor로 둔다. 필요해질 때만 actor를 방법 B로 승격한다. -->

## 3. Track and waypoint layout
<!-- 3. 트랙과 웨이포인트 배치 -->

Keep every track as an ordered waypoint list in one YAML file — the single source of truth read by both the world-building step and the waypoint-follower node:
<!-- 모든 트랙은 하나의 YAML 파일에 순서 있는 웨이포인트 목록으로 관리한다 — 월드 구성과 웨이포인트 팔로워 노드가 함께 읽는 단일 기준. -->

Do not type waypoint coordinates by hand — author them by clicking on the occupancy map with `tools/track_editor.py`, which reads and writes this YAML (yaw is derived automatically from the segment direction):
<!-- 웨이포인트 좌표를 손으로 입력하지 말 것 — tools/track_editor.py에서 occupancy 맵을 클릭해 작성한다. 편집기가 이 YAML을 읽고 쓰며, yaw는 구간 방향에서 자동 계산된다. -->

```bash
.venv/bin/python tools/track_editor.py \
  --map src/aws-robomaker-small-warehouse-world-ros2/maps/005/map.yaml \
  --out config/tracks.yaml
# left click = add, right click = undo, n = new track, t = switch, l = toggle loop, s = save
# p = place signal point, m = add station point, d = delete nearest station
```

Besides tracks, the same editor places the named points: the signal point (`p`) and station points (`m`) — the spots vehicles visit and return from. They are saved into the same YAML under `signal_point` and `stations`.
<!-- 같은 편집기로 트랙 외의 지점들도 찍는다: 수신호 포인트(p)와 운송장비가 찍고 돌아오는 경유 지점(m). 같은 YAML의 signal_point / stations 항목에 저장된다. -->

None of this requires regenerating the occupancy map — tracks and points are logical overlays on top of it; the map itself only changes when physical obstacles are added or moved (see §8).
<!-- 이 작업에는 occupancy 맵 재생성이 전혀 필요 없다 — 트랙과 지점은 맵 위의 논리적 오버레이일 뿐이고, 맵 자체는 물리적 장애물이 추가·이동될 때만 다시 만든다 (8장 참고). -->

The tools need only Python + matplotlib/Pillow/PyYAML (no ROS, no Gazebo). Run them from the repo-root `.venv` (Python 3.12, same interpreter version as Signal-Vision) — create it once per machine:
<!-- 툴은 Python + matplotlib/Pillow/PyYAML만 있으면 된다(ROS·Gazebo 불필요). 레포 루트의 .venv(Python 3.12, Signal-Vision과 같은 인터프리터 버전)에서 실행하며, 머신마다 한 번만 만들면 된다: -->

```bash
python3.12 -m venv .venv          # on the Ubuntu VM plain python3 is already 3.12
.venv/bin/pip install -r requirements.txt
```

Prefix every tool invocation with `.venv/bin/python` (as in the commands above and in §4–5) instead of relying on whatever `python3` the shell happens to have.
<!-- 셸에 잡혀 있는 아무 python3에 의존하지 말고, 모든 툴 실행은 (위와 4~5장의 명령처럼) .venv/bin/python으로 시작할 것. -->

```yaml
# config/tracks.yaml  (poses are x, y, yaw in world frame; tune on the real map)
signal_point: [0.0, -4.0, 1.5708]

tracks:
  patrol_a:            # loop track for background vehicle(s)
    loop: true
    waypoints:
      - [ 4.0, -6.0, 0.0]
      - [ 4.0,  6.0, 1.5708]
      - [-4.0,  6.0, 3.1416]
      - [-4.0, -6.0, -1.5708]
  approach:            # the single track that converges on the signal point
    loop: false
    waypoints:
      - [ 6.0, -8.0, 3.1416]
      - [ 2.0, -8.0, 3.1416]
      - [ 0.0, -6.0, 1.5708]
      - [ 0.0, -4.0, 1.5708]   # ends at signal_point
```

- The coordinates above are placeholders — read real corridor positions from the running world (§4) before committing them.
  <!-- 위 좌표는 자리표시자다 — 확정 전에 실행 중인 월드에서 실제 통로 좌표를 읽어와야 한다 (4장 참고). -->
- For a seamless loop, make the path from the last waypoint back to the first collision-free; for actors, repeat the first pose as the final waypoint.
  <!-- 끊김 없는 순환을 위해 마지막 웨이포인트에서 첫 웨이포인트로 돌아가는 경로에 장애물이 없어야 하고, actor는 첫 pose를 마지막 웨이포인트로 한 번 더 넣는다. -->
- Run multiple vehicles on the same track by offsetting their start time (actors: shift every `<time>`; models: start each follower at a different waypoint index).
  <!-- 같은 트랙에 여러 대를 돌리려면 시작 시점을 어긋나게 한다 (actor는 모든 <time>을 밀고, 모델은 팔로워의 시작 웨이포인트 인덱스를 다르게 준다). -->

## 4. Object placement in the world
<!-- 4. 월드에 객체 배치하기 -->

- Permanent placement is done in the world SDF with `<include>` + `<pose>`; GUI-placed objects vanish when the simulation closes.
  <!-- 영구 배치는 월드 SDF에서 <include> + <pose>로 한다. GUI로 배치한 객체는 시뮬레이션을 끄면 사라진다. -->

```xml
<include>
  <uri>model://aws_robomaker_warehouse_ShelfF_01</uri>
  <name>shelf_track_edge_01</name>   <!-- unique per instance -->
  <pose>3.0 -2.0 0 0 0 1.5708</pose> <!-- x y z roll pitch yaw (rad) -->
</include>
```

- Workflow for finding poses: drag the object roughly in the GUI (T = translate, R = rotate), then read the exact values from Entity Tree → Component Inspector → Pose, and copy them into the SDF / `tracks.yaml`.
  <!-- 좌표 잡는 흐름: GUI에서 대충 끌어다 놓고(T 이동, R 회전) Entity Tree → Component Inspector → Pose에서 정확한 값을 읽어 SDF와 tracks.yaml에 옮겨 적는다. -->
- Placement rules for tracks to work:
  <!-- 트랙이 제대로 돌기 위한 배치 규칙: -->
  - Keep every track corridor at least vehicle width + 0.5 m clear of static objects, and wider at corners.
    <!-- 모든 트랙 통로는 정적 객체로부터 최소 '장비 폭 + 0.5 m'의 여유를 두고, 코너는 더 넓게 확보한다. -->
  - Keep the area around the signal point open so the camera has a clear line of sight to the signal worker.
    <!-- 수신호 포인트 주변은 카메라가 신호수를 가리는 것 없이 볼 수 있도록 비워 둔다. -->
  - Scenery that never moves should include `<static>true</static>` (the vendored warehouse models already set this) so physics skips it.
    <!-- 움직일 일 없는 배경 객체는 <static>true</static>로 두어 물리 연산에서 제외한다 (벤더링된 창고 모델들은 이미 설정돼 있음). -->
  - Spawn Method B vehicles directly on their track, at their starting waypoint pose, so the follower does not start with a large correction maneuver.
    <!-- 방법 B 장비는 시작 웨이포인트 pose 그대로 트랙 위에 스폰해서, 팔로워가 시작부터 큰 보정 기동을 하지 않게 한다. -->

To make the layout visible on the warehouse floor, generate painted markers from `tracks.yaml` with `tools/tracks_to_markers.py`: a colored line strip per track, a red disc at the signal point, and a green disc per station. The visuals have no collision bodies, so physics and LiDAR are unaffected. Injection is idempotent (re-running replaces the generated block), same as the actor generator in §5.
<!-- 배치를 창고 바닥에 눈에 보이게 하려면 tools/tracks_to_markers.py로 tracks.yaml에서 도색 마커를 생성한다: 트랙마다 색 라인, 수신호 포인트에 빨간 원판, 경유 지점마다 초록 원판. 충돌체가 없는 시각 전용이라 물리·라이다에는 영향이 없다. 주입은 5장의 actor 생성기와 마찬가지로 재실행 시 기존 블록을 교체하는 멱등 방식이다. -->

```bash
.venv/bin/python tools/tracks_to_markers.py --tracks config/tracks.yaml \
  --inject src/aws-robomaker-small-warehouse-world-ros2/worlds/navi_factory/navi_factory.world
```

## 5. Method A — background vehicle as an actor
<!-- 5. 방법 A — 배경 운송장비를 actor로 -->

Do not write actor SDF by hand — generate it from `tracks.yaml` with `tools/tracks_to_actors.py`, which emits one actor per loop track and can write it straight into a world file (idempotent: re-running replaces the previously generated block between its markers):
<!-- actor SDF를 손으로 쓰지 말 것 — tools/tracks_to_actors.py로 tracks.yaml에서 생성한다. loop 트랙마다 actor를 하나씩 만들어 월드 파일에 바로 써 넣을 수 있고, 다시 실행하면 마커 사이의 기존 생성 블록을 교체하므로 반복 실행해도 안전하다. -->

```bash
.venv/bin/python tools/tracks_to_actors.py --tracks config/tracks.yaml --speed 0.75 \
  --inject src/aws-robomaker-small-warehouse-world-ros2/worlds/navi_factory/navi_factory.world
```

The generated SDF looks like this (shown for reference; times correspond to ~0.75 m/s):
<!-- 생성되는 SDF는 아래와 같다 (참고용이며, 시간 값은 약 0.75 m/s 기준). -->

```xml
<actor name="patrol_vehicle_1">
  <link name="body">
    <visual name="visual">
      <geometry>
        <!-- reuse the warehouse pallet-jack mesh; or start with <box><size>1.2 0.8 0.5</size></box> -->
        <mesh>
          <uri>model://aws_robomaker_warehouse_PalletJackB_01/meshes/aws_robomaker_warehouse_PalletJackB_01_visual.DAE</uri>
        </mesh>
      </geometry>
    </visual>
  </link>
  <script>
    <loop>true</loop>
    <auto_start>true</auto_start>
    <trajectory id="0" type="square">
      <waypoint><time>0</time>  <pose>4 -6 0 0 0 0</pose></waypoint>
      <waypoint><time>16</time> <pose>4 6 0 0 0 0</pose></waypoint>
      <waypoint><time>17</time> <pose>4 6 0 0 0 1.5708</pose></waypoint>
      <waypoint><time>28</time> <pose>-4 6 0 0 0 1.5708</pose></waypoint>
      <waypoint><time>29</time> <pose>-4 6 0 0 0 3.1416</pose></waypoint>
      <waypoint><time>45</time> <pose>-4 -6 0 0 0 3.1416</pose></waypoint>
      <waypoint><time>46</time> <pose>-4 -6 0 0 0 -1.5708</pose></waypoint>
      <waypoint><time>57</time> <pose>4 -6 0 0 0 -1.5708</pose></waypoint>
      <waypoint><time>58</time> <pose>4 -6 0 0 0 0</pose></waypoint>
    </trajectory>
  </script>
</actor>
```

- Check the actual mesh filename under `src/aws-robomaker-small-warehouse-world-ros2/models/aws_robomaker_warehouse_PalletJackB_01/meshes/` before using it.
  <!-- 사용 전에 해당 모델 meshes/ 폴더에서 실제 메시 파일명을 확인할 것. -->
- Waypoints are interpolated linearly in time; the extra 1-second waypoints at corners rotate the vehicle in place so it faces its direction of travel.
  <!-- 웨이포인트 사이는 시간에 대해 선형 보간된다. 코너의 1초짜리 추가 웨이포인트는 제자리 회전으로 진행 방향을 맞춰 주는 역할이다. -->

## 6. Method B — LiDAR-carrying vehicle (model + DiffDrive + follower node)
<!-- 6. 방법 B — 라이다 장착 운송장비 (모델 + DiffDrive + 팔로워 노드) -->

The vehicle is a normal SDF model (chassis + two driven wheels + casters) with the diff-drive plugin:
<!-- 장비는 일반 SDF 모델(차체 + 구동 바퀴 2개 + 캐스터)이며 diff-drive 플러그인을 단다: -->

```xml
<model name="transport_1">
  <pose>6 -8 0.1 0 0 3.1416</pose>  <!-- spawn at the first approach waypoint -->
  <!-- ... chassis link, wheel links and joints ... -->
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.6</wheel_separation>
    <wheel_radius>0.15</wheel_radius>
    <topic>/model/transport_1/cmd_vel</topic>
    <odom_topic>/model/transport_1/odometry</odom_topic>
  </plugin>
</model>
```

Bridge the topics to ROS 2 (`ros_gz_bridge` is already installed):
<!-- 토픽을 ROS 2로 브리지한다 (ros_gz_bridge는 이미 설치돼 있음): -->

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/transport_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/transport_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /model/transport_1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

The waypoint-follower is a small Python node (to live in a future package here): it loads `tracks.yaml`, compares the current odometry pose to the next waypoint, publishes `cmd_vel` (turn toward the waypoint, drive, advance to the next one), and wraps around when `loop: true`.
<!-- 웨이포인트 팔로워는 (추후 이 레포의 패키지에 들어갈) 작은 Python 노드다: tracks.yaml을 읽고, 현재 odometry pose와 다음 웨이포인트를 비교해 cmd_vel을 발행하며(웨이포인트 방향으로 회전 → 전진 → 다음 웨이포인트로), loop: true면 처음으로 되돌아간다. -->

This is also the integration point for the hand-signal commands: when the vehicle reaches the signal point, the follower pauses and yields control to the stop / turn-left / turn-right commands arriving from Signal-Vision over ROS 2.
<!-- 이 노드가 수신호 명령의 연동 지점이기도 하다: 장비가 수신호 포인트에 도착하면 팔로워는 정지하고, Signal-Vision에서 ROS 2로 들어오는 정지/좌회전/우회전 명령에 제어권을 넘긴다. -->

## 7. Mounting the LiDAR
<!-- 7. 라이다 장착 -->

Add a dedicated link fixed to the chassis, with the sensor on it (inside the Method B model):
<!-- 차체에 고정된 전용 링크를 만들고 그 위에 센서를 단다 (방법 B 모델 내부에): -->

```xml
<link name="lidar_link">
  <pose relative_to="chassis">0.3 0 0.45 0 0 0</pose>  <!-- above the cargo bed, clear of the body -->
  <inertial><mass>0.1</mass></inertial>
  <sensor name="lidar" type="gpu_lidar">
    <topic>/model/transport_1/scan</topic>
    <update_rate>5</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>180</samples>
          <min_angle>-3.1416</min_angle>
          <max_angle>3.1416</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
      </range>
    </lidar>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</link>
<joint name="lidar_joint" type="fixed">
  <parent>chassis</parent>
  <child>lidar_link</child>
</joint>
```

Requirements and caveats:
<!-- 요구 사항과 주의점: -->

- The world must load the sensors system — both warehouse worlds here already have `gz-sim-sensors-system` with `<render_engine>ogre2</render_engine>`, which is required: `gpu_lidar` only works on ogre2. Do not change that line even though the GUI runs with `--render-engine ogre`.
  <!-- 월드에 sensors 시스템 플러그인이 있어야 한다 — 이 레포의 두 창고 월드에는 이미 gz-sim-sensors-system이 ogre2로 들어 있고, gpu_lidar는 ogre2에서만 동작하므로 GUI를 --render-engine ogre로 돌리더라도 그 줄은 바꾸지 말 것. -->
- `gpu_lidar` renders the scene every scan, which is expensive without GPU acceleration. In the QEMU VM keep `update_rate` ≤ 5 Hz and `samples` ≤ 180 until GPU acceleration works, then raise them.
  <!-- gpu_lidar는 스캔마다 씬을 렌더링해서 GPU 가속이 없으면 부하가 크다. QEMU VM에서는 GPU 가속이 되기 전까지 update_rate ≤ 5 Hz, samples ≤ 180으로 두고, 이후에 올릴 것. -->
- Place the sensor pose above/outside the vehicle body, or the scan will hit the vehicle itself; verify with `<visualize>true</visualize>` (rays drawn in the GUI).
  <!-- 센서 pose는 차체 위/바깥에 두어야 스캔이 자기 차체를 때리지 않는다. <visualize>true</visualize>로 GUI에서 레이를 그려 확인할 것. -->
- Mounting a LiDAR on an actor does nothing (see §2) — this section only applies to Method B vehicles.
  <!-- actor에 단 라이다는 동작하지 않는다 (2장 참고) — 이 장은 방법 B 장비에만 해당한다. -->

## 8. Extracting a navigable area ("navmesh")
<!-- 8. 주행 가능 영역("navmesh") 추출 -->

Gazebo itself has no navmesh concept. There are two practical routes:
<!-- Gazebo 자체에는 navmesh 개념이 없다. 실용적인 경로는 두 가지다: -->

**Route 1 (recommended) — 2D occupancy grid, the ROS-native equivalent.** The vendored warehouse package already ships a ready-made map matching `navi_factory`'s layout: `src/aws-robomaker-small-warehouse-world-ros2/maps/005/map.yaml` (PGM + YAML, standard `map_server` format). Use this with Nav2 for path planning; this is what Phase 2 obstacle-avoidance should build on.
<!-- 경로 1 (권장) — ROS 표준 방식인 2D occupancy grid. 벤더링된 창고 패키지에 navi_factory 배치와 맞는 완성된 맵이 이미 있다(maps/005의 PGM+YAML, map_server 표준 형식). Nav2 경로 계획에 이걸 쓰면 되고, Phase 2 장애물 회피도 이 위에 쌓으면 된다. -->

If the world layout changes (added shelves, new tracks), regenerate the map by driving a robot with the mounted LiDAR around while running `slam_toolbox`, then save with `ros2 run nav2_map_server map_saver_cli -f maps/custom/map`.
<!-- 월드 배치가 바뀌면(선반 추가, 새 트랙 등) 라이다 장착 로봇을 slam_toolbox를 켠 채로 몰고 다닌 뒤 map_saver_cli로 저장해서 맵을 다시 만든다. -->

**Route 2 — a true 3D navmesh, only if an external (non-ROS) planner needs one.** Export the model meshes (`models/*/meshes/*.DAE`) into Blender, merge the floor and obstacles into one mesh, export as OBJ, and bake the navmesh with recastnavigation's RecastDemo (adjust agent radius/height to the vehicle size). Nothing in the current ROS 2 pipeline consumes a navmesh, so skip this route unless a specific tool demands it.
<!-- 경로 2 — 진짜 3D navmesh는 ROS 외부 플래너가 요구할 때만. 모델 메시(models/*/meshes/*.DAE)를 Blender로 모아 바닥+장애물을 하나의 메시로 합치고 OBJ로 내보낸 뒤 recastnavigation의 RecastDemo로 굽는다(agent 반경/높이를 장비 크기에 맞출 것). 현재 ROS 2 파이프라인에는 navmesh를 쓰는 곳이 없으므로, 특정 도구가 요구하지 않는 한 이 경로는 생략한다. -->

The waypoint tracks in §3 do not need a navmesh at all — they are hand-authored fixed routes; a navmesh/costmap only becomes relevant when vehicles must plan their own paths (Phase 2).
<!-- 3장의 웨이포인트 트랙에는 navmesh가 아예 필요 없다 — 손으로 지정한 고정 경로이기 때문이다. navmesh/costmap은 장비가 스스로 경로를 계획해야 할 때(Phase 2)부터 의미가 있다. -->

## 9. Suggested layout for the upcoming package
<!-- 9. 추후 패키지의 권장 구조 -->

```
src/signal_transport/            # new ROS 2 Python package (name TBD)
├── config/
│   └── tracks.yaml              # §3 single source of truth
├── worlds/
│   └── warehouse_tracks.sdf     # warehouse world + actors + vehicle includes
├── models/
│   └── transport_vehicle/       # Method B vehicle (chassis, wheels, lidar)
├── launch/
│   └── sim.launch.py            # gz sim + bridge + follower node(s)
└── signal_transport/
    └── waypoint_follower.py     # §6 node (snake_case, Python 3.12)
```

When this scaffolding is actually created, update `CLAUDE.md` with the real build/run commands per its own instructions.
<!-- 이 스캐폴딩을 실제로 만들면, CLAUDE.md의 자체 지침대로 실제 빌드/실행 명령을 CLAUDE.md에 반영할 것. -->
