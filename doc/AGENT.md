# AGENT.md — Project structure

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. It covers the **concrete code structure**: workspace layout, packages, files, the node/topic graph, and build/run commands. For project purpose and current status see `doc/design.md`; for the ML training process see `doc/ml-flow.md`.

**Note on the Korean lines in this file:** they are HTML comments (`<!-- -->`) placed only for Korean-speaking human readers. They are not additional instructions — do not parse, translate, or derive behavior from them. Treat the English text as the sole operative content.

This English-body + Korean-comment pattern applies to any bilingual doc in this repo, not just this file.
<!-- 이 영어 본문 + 한글 주석 패턴은 이 파일뿐 아니라 리포지토리 내 모든 이중언어 문서에 적용됨. -->

Before finalizing a bilingual doc, re-read the diff and verify two things:
<!-- 이중언어 문서를 마무리하기 전에, diff를 다시 읽고 두 가지를 검증할 것: -->

- Every operative English line has its paired Korean comment — including lines synthesized fresh from conversation, not only ones translated from pre-existing Korean text.
  <!-- 모든 영어 본문 줄에 짝이 되는 한글 주석이 있는지 — 기존 한글을 번역한 줄뿐 아니라, 대화에서 새로 종합해 쓴 줄도 포함해서. -->
- No Korean word has leaked into the English body itself.
  <!-- 영어 본문 안에 한글 단어가 섞여 들어가지 않았는지. -->

A new bilingual doc needs its own copy of this whole note at the top — this note is self-referential to this file and does not automatically extend to other files.
<!-- 새 이중언어 문서를 만들 때는 이 노트 전체를 그 파일 맨 위에도 따로 복사해 넣어야 함 — 이 노트는 "이 파일"에 대해서만 자기지시적이라 다른 파일에 자동으로 적용되지 않음. -->

## Workspace layout

<!-- 워크스페이스 구성 -->

This is a colcon (ROS 2 Jazzy) workspace on Gazebo Harmonic (`gz sim`). ROS 2 packages live under `src/`; `build/`, `install/`, `log/` are build artifacts and are gitignored.
<!-- ROS 2 Jazzy + Gazebo Harmonic(gz sim) 기반 colcon 워크스페이스임. ROS 2 패키지는 src/ 아래에 있고, build/ · install/ · log/ 는 빌드 산출물이라 gitignore됨. -->

```
Signal-Simulation/
├── CLAUDE.md              # personal local prompt (gitignored); @imports doc/*.md
├── README.md              # short public description of the repo
├── doc/                   # team-shared prompts (committed)
│   ├── AGENT.md           # this file — project structure
│   ├── design.md          # project overview & current status
│   ├── ml-flow.md         # machine-learning training process (currently empty)
│   └── work.md            # work log (currently empty)
├── docs/                  # long-form working guides (committed, bilingual)
│   ├── camera_node_refactory.md    # camera_node responsibility split & handoff notes
│   └── gazebo-control-guide.md     # moving transport equipment on tracks
├── examples/              # separate scratch workspace of ROS 2 practice packages
├── scripts/               # build/run entry points + external-repo source vendoring
├── src/
│   ├── robot_control/     # the robot's body and wiring: urdf + config, no nodes
│   ├── signal_vision/     # gesture recognition: camera_node + the vendored model source
│   ├── auto_drive/        # autonomous driving: mission_follower (+ legacy waypoint_follower) + scan_rays viewer
│   └── knavi_bringup/     # assembly only: the top-level launch, no nodes of its own
├── tools/                 # offline track editing / SDF generation (see requirements.txt)
└── worlds/
    └── navi_factory/      # the world bringup uses
        ├── models/        # warehouse props
        └── world/navi_factory/
            ├── navi_factory.sdf   # the world itself
            └── tracks.yaml        # dispatch routes & zone coords — single source of truth
```
<!-- 위 트리: 루트에 개인 로컬 CLAUDE.md와 README, 팀 공용 doc/ 와 docs/, 연습용 examples/, 실행 스크립트 scripts/, ROS 2 패키지 4개가 든 src/, 오프라인 도구 tools/, 그리고 월드 worlds/navi_factory (월드 SDF와 tracks.yaml이 같은 폴더에 있음). -->

Four layout facts that differ from a plain ROS 2 workspace:
<!-- 일반적인 ROS 2 워크스페이스와 다른 네 가지: -->

- The world is **not** a ROS 2 package and does not live under `src/`. `worlds/navi_factory/models/` holds the warehouse props (AWS RoboMaker warehouse models, OGV, cones, carts, shelves) and `worlds/navi_factory/world/navi_factory/navi_factory.sdf` is the world itself. Gazebo finds them through `GZ_SIM_RESOURCE_PATH`, which the launch file sets from the workspace root.
  <!-- 월드는 ROS 2 패키지가 아니고 src/ 아래에 있지도 않음. worlds/navi_factory/models/ 에 창고 소품(AWS 창고 모델, OGV, 콘, 카트, 선반)이 있고, 월드 본체는 worlds/navi_factory/world/navi_factory/navi_factory.sdf. Gazebo는 launch가 워크스페이스 루트 기준으로 설정하는 GZ_SIM_RESOURCE_PATH로 이들을 찾음. -->
- The previously vendored `aws-robomaker-small-warehouse-world-ros2` package is gone from `src/`; its models were folded into `worlds/navi_factory/models/`.
  <!-- 예전에 벤더링해 두었던 aws-robomaker-small-warehouse-world-ros2 패키지는 src/ 에서 사라졌고, 그 모델들은 worlds/navi_factory/models/ 안으로 흡수됨. -->
- `tracks.yaml` sits **next to the world SDF**, not in any package. Its coordinates are measured off `navi_factory.sdf`'s `track_*` tile poses, and `tools/tracks_to_{markers,actors}.py` generate discs and actors from it straight back into that same SDF — it is a world-bound layout description, not a package's tuning parameters. Two packages read it (`auto_drive` for routes, `signal_vision` for the gesture-zone gate), so neither can own it; the launch file resolves the path and passes it to both as the `tracks_yaml_path` parameter.
  <!-- tracks.yaml 은 어떤 패키지도 아닌 월드 SDF 옆에 있음. 좌표가 navi_factory.sdf 의 track_* 타일 pose 실측값이고, tools/tracks_to_{markers,actors}.py 가 이 파일에서 다시 그 SDF 로 원판과 actor 를 생성해 넣기 때문 — 월드에 종속된 배치 기술서지 특정 패키지의 튜닝 파라미터가 아님. 읽는 쪽이 둘(auto_drive 는 경로, signal_vision 은 수신호 존 게이트)이라 어느 쪽도 소유할 수 없고, launch 가 경로를 계산해 tracks_yaml_path 파라미터로 양쪽에 넘김. -->
- The package that owns a file and the package that reads it are usually different. `knavi_bringup` holds the launch but no urdf and no config; `robot_control` holds urdf and config but starts nothing. Expect to edit two packages for one change.
  <!-- 파일을 가진 패키지와 그 파일을 읽는 패키지가 대체로 다름. knavi_bringup 은 launch 만 갖고 urdf·config 는 없으며, robot_control 은 urdf·config 를 갖지만 아무것도 실행하지 않음. 변경 하나에 패키지 두 개를 고치게 되는 걸 예상할 것. -->

## Packages

<!-- 패키지들 -->

Four `ament_python` packages, split by responsibility rather than by file type. Every one of them excludes the world — the world is injected through the launch `world` argument, so no package installs world files.
<!-- ament_python 패키지 4개이며, 파일 종류가 아니라 책임 단위로 나뉘어 있음. 넷 모두 월드는 제외함 — 월드는 launch의 world 인자로 주입하므로 어느 패키지도 월드 파일을 설치하지 않음. -->

| Package | Owns | Runs |
|---|---|---|
| `robot_control` | `urdf/` (`mecanum_lift_robot.urdf.xacro` is the one in use; `robot.urdf.xacro` sits next to it but nothing reads it), `config/` (`bridge.yaml`, `twist_mux.yaml`, `flat_ground.sdf`) | nothing — no console script, no launch |
| `signal_vision` | `camera_node/`, the vendored `vision_hand/`, `metrics.py`, `models/` (weights) | `camera_node` |
| `auto_drive` | `patrol/mission_follower.py`, `patrol/waypoint_follower.py`, `viz/scan_rays.py` | `mission_follower`, `waypoint_follower` (legacy), `scan_rays` |
| `knavi_bringup` | `launch/bringup.launch.py` | nothing of its own — it starts the other three |
<!-- 위 표: 패키지 4개가 각각 무엇을 소유하고 무엇을 실행하는지. robot_control 과 knavi_bringup 은 실행 노드가 없음. -->

Because `robot_control` executes nothing, its `package.xml` declares only `xacro` — the one tool without which its own files cannot even be read. Everything needed to actually start nodes (`rclpy`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `twist_mux`, the message packages) is declared by `knavi_bringup`, which is what runs them.
<!-- robot_control 은 실행하는 게 없으므로 package.xml 에 xacro 만 선언함 — 그것 없이는 자기 파일조차 못 읽는 유일한 도구이기 때문. 노드를 실제로 띄우는 데 필요한 것들(rclpy, robot_state_publisher, ros_gz_sim, ros_gz_bridge, twist_mux, 메시지 패키지들)은 그것들을 실행하는 knavi_bringup 이 선언함. -->

Each package also carries `test/`, which is style checks only: `ament_copyright`, `ament_flake8`, `ament_pep257`.
<!-- 각 패키지에 test/ 도 있는데 내용은 스타일 검사뿐임: ament_copyright, ament_flake8, ament_pep257. -->

### `camera_node` file split

<!-- camera_node 파일 분리 -->

One node, one process, several files. `node.py` only wires the parts together and owns the threads; it imports neither `cv2` nor `Twist`.
<!-- 노드 하나, 프로세스 하나인데 파일만 여러 개임. node.py는 부품을 배선하고 스레드만 관리하며, cv2도 Twist도 직접 import하지 않음. -->

- `node.py` — assembly, parameters, one capture+inference worker thread and one render worker thread
  <!-- node.py — 조립, 파라미터, 캡처+추론 워커 스레드 1개와 렌더 워커 스레드 1개 -->
- `frame_source.py` — webcam capture (`v4l2` by default; `any`, `avfoundation`, `dshow` also selectable)
  <!-- frame_source.py — 웹캠 캡처 (기본 v4l2, any·avfoundation·dshow도 선택 가능) -->
- `image_publisher.py` — publishes `image_webcam` (`sensor_msgs/Image`), disabled by default
  <!-- image_publisher.py — image_webcam(sensor_msgs/Image) 발행, 기본은 꺼져 있음 -->
- `inference.py` — runs the vendored gesture model and produces an immutable render payload
  <!-- inference.py — 벤더링된 수신호 모델을 돌리고, 렌더용 불변 묶음(payload)을 만들어 냄 -->
- `command_publisher.py` — maps a label to `gesture` (`std_msgs/String`), motion labels to `cmd_vel_gesture` (`geometry_msgs/Twist`), and dispatch labels to `signal_dispatch` (`std_msgs/String`, a zone name)
  <!-- command_publisher.py — 라벨을 gesture(std_msgs/String)로, 속도 라벨은 cmd_vel_gesture(geometry_msgs/Twist)로, 파견 라벨은 signal_dispatch(std_msgs/String, 존 이름)로 변환해 발행 -->
- `labels.py` — the single source of truth for labels: motion `STOP` (the only one left), dispatch `LEFT`→`zone_nw` / `RIGHT`→`zone_ne`
  <!-- labels.py — 라벨의 단일 출처: 속도 라벨 STOP(이제 이것 하나뿐), 파견 라벨 LEFT→zone_nw / RIGHT→zone_ne -->
- `shutdown.py`, `swap_frame.py` — shutdown detection and frame handoff helpers
  <!-- shutdown.py, swap_frame.py — 종료 감지와 프레임 인계용 헬퍼 -->

`signal_vision/vision_hand/` is a vendored copy produced by `scripts/setup_infer_env.py`, not code owned by this repo. Refresh it from the source repository instead of editing it here; see `doc/design.md`.
<!-- signal_vision/vision_hand/ 는 scripts/setup_infer_env.py 가 복사해 넣은 벤더 사본이며 이 리포지토리가 소유한 코드가 아님. 여기서 고치지 말고 원본 리포지토리에서 다시 복사해 갱신할 것 — doc/design.md 참고. -->

That directory is excluded from the linters, because enforcing this repo's style on someone else's code is pointless when the next refresh overwrites it. The exclusion needs two mechanisms: `ament_flake8` honours the `AMENT_IGNORE` marker file that `setup_infer_env.py` re-creates after every copy, while `ament_pep257` ignores that marker and is told through `--exclude` in `signal_vision/test/test_pep257.py`.
<!-- 그 디렉터리는 린터 대상에서 빠져 있음. 다음 갱신 때 덮일 남의 코드에 이 리포지토리의 스타일을 강제해 봐야 의미가 없기 때문. 제외에는 방식이 두 개 필요함: ament_flake8 은 setup_infer_env.py 가 복사할 때마다 다시 만드는 AMENT_IGNORE 마커 파일을 인식하는 반면, ament_pep257 은 그 마커를 무시하므로 signal_vision/test/test_pep257.py 의 --exclude 로 알려 줌. -->

## Node graph

<!-- 노드 그래프 -->

What `knavi_bringup/launch/bringup.launch.py` actually starts. One robot is spawned today: `robot2`, the `mecanum_lift_robot`. `robot1` was removed from the `robot_info` list because running two physical robots in this mesh-heavy world costs too much, and `robot2` is the one carrying the lidar and camera.
<!-- knavi_bringup/launch/bringup.launch.py 가 실제로 띄우는 것. 현재 스폰되는 로봇은 하나임: robot2, 즉 mecanum_lift_robot. robot1 은 robot_info 목록에서 빠졌는데, 메시가 많은 이 월드에서 물리 로봇 두 대를 돌리면 부담이 크고, 라이다와 카메라를 단 쪽이 robot2 이기 때문. -->

| Component | Count | Lifetime | Role |
|---|---|---|---|
| `robot_state_publisher` (ns `/robot2`) | one per robot | resident | expands URDF, publishes link tf and `robot_description` |
| `ros_gz_sim create` (robot) | one per robot | one-shot | reads `robot_description` from the topic and spawns the model |
| `ros_gz_sim create` (`flat_ground`) | one | one-shot | spawns the collision-only ground plate at z = 0.25, unless `enable_flat_ground:=false` |
| `twist_mux` (ns `/robot2`) | one per robot | resident | passes through the highest-priority live velocity source |
| `mission_follower` (ns `/robot2`) | one per robot | resident | waits at the signal point, drives a `tracks.yaml` route to the zone named on `signal_dispatch` and returns; publishes `cmd_vel_auto`; off with `enable_patrol:=false` |
| `camera_node` (ns `/robot2`) | one per robot | resident | one physical webcam per robot; off with `enable_camera:=false` |
| `scan_rays` (ns `/robot2`) | one per robot | resident | redraws `scan` as ray line segments on `scan_rays`; starts only when RViz does |
| `ros_gz_bridge bridge_node` | one for the whole system | resident | translates the topics listed in `config/bridge.yaml` |
| `rviz2` | one for the whole system | resident | views `/tf`, `<ns>/scan` and `<ns>/scan_rays`; off with `enable_rviz:=false` or `headless:=true` |
| `gz sim` process | one | resident | physics; **not** a ROS node, so it never appears in `ros2 node list` |
<!-- 위 표: bringup이 띄우는 구성요소별 개수·수명·역할. gz sim은 ROS 노드가 아니라 별도 프로세스라 ros2 node list에 안 나옴. -->

Inside the Gazebo process, not separate nodes: the `DiffDrive` plugin (subscribes `cmd_vel`, publishes `odom` and `odom → base_link` tf), the `JointStatePublisher` plugin, the simulated camera sensor, and the `gpu_lidar` sensor.
<!-- Gazebo 프로세스 내부에서 도는 것들(별도 노드가 아님): DiffDrive 플러그인(cmd_vel 구독, odom과 odom→base_link tf 발행), JointStatePublisher 플러그인, 시뮬레이션 카메라 센서, gpu_lidar 센서. -->

Startup order: set `GZ_SIM_RESOURCE_PATH` → `gz sim` → `robot_state_publisher` → spawn → ground plate → bridge → `camera_node` → `rviz2`.
<!-- 기동 순서: GZ_SIM_RESOURCE_PATH 설정 → gz sim → robot_state_publisher → 스폰 → 바닥판 → 브릿지 → camera_node → rviz2. -->

Adding a robot means adding one entry to the `robot_info` list at the top of the launch file; the bridge and `gz sim` stay at one each, and only `bridge.yaml` grows.
<!-- 로봇을 늘리려면 launch 파일 상단 robot_info 리스트에 항목을 추가하면 됨. 브릿지와 gz sim은 계속 1개씩이고 bridge.yaml에 토픽만 늘어남. -->

`camera_node` scales with the robots too: one webcam per robot, each node inside that robot's namespace. That is what makes the wiring self-consistent — a namespaced node publishes `<ns>/cmd_vel_gesture`, which the `twist_mux` in the same namespace already subscribes to, so no remap is involved and there is no second place for the two to drift apart.
<!-- camera_node 도 로봇 수에 따라 늘어남. 로봇 한 대에 웹캠 한 대이고, 각 노드는 그 로봇의 네임스페이스 안에 있음. 배선이 저절로 맞아떨어지는 이유가 이것임 — 네임스페이스가 붙은 노드는 <ns>/cmd_vel_gesture 로 발행하고 같은 네임스페이스의 twist_mux 가 이미 그걸 구독하므로, 리맵이 끼어들지 않고 둘이 어긋날 두 번째 자리도 없음. -->

Which webcam goes with which robot is the fifth field of a `robot_info` entry, a `/dev/video<N>` index. The `camera_device_id` launch argument overrides that field for the first robot only, and defaults to empty so that leaving it alone keeps every robot on its own configured value.
<!-- 어느 웹캠이 어느 로봇에 붙는지는 robot_info 항목의 다섯 번째 필드이며 /dev/video<N> 의 번호임. camera_device_id launch 인자는 첫 번째 로봇의 그 필드만 덮어쓰고, 기본값이 비어 있어서 인자를 건드리지 않으면 모든 로봇이 각자 설정된 값을 그대로 씀. -->

Running N robots therefore needs N webcams physically plugged in. When a device is missing only that robot's `camera_node` fails; the other robots and the simulation keep running.
<!-- 따라서 로봇을 N대 돌리려면 실물 웹캠도 N대가 꽂혀 있어야 함. 장치가 없으면 그 로봇의 camera_node 만 실패하고, 나머지 로봇과 시뮬레이션은 계속 돎. -->

## Topics

<!-- 토픽 -->

Bridged between ROS 2 and Gazebo (`config/bridge.yaml`):
<!-- ROS 2와 Gazebo 사이에서 브리지되는 토픽 (config/bridge.yaml): -->

| Topic | Direction | ROS type | Gazebo type |
|---|---|---|---|
| `/clock` | gz → ROS | `rosgraph_msgs/Clock` | `gz.msgs.Clock` |
| `<ns>/cmd_vel` | **ROS → gz** | `geometry_msgs/Twist` | `gz.msgs.Twist` |
| `<ns>/odom` | gz → ROS | `nav_msgs/Odometry` | `gz.msgs.Odometry` |
| `<ns>/joint_states` | gz → ROS | `sensor_msgs/JointState` | `gz.msgs.Model` |
| `<ns>/scan` | gz → ROS | `sensor_msgs/LaserScan` | `gz.msgs.LaserScan` |
| `/tf` (gz side `<ns>/tf`) | gz → ROS | `tf2_msgs/TFMessage` | `gz.msgs.Pose_V` |
| `/robot2/pose_gt` | gz → ROS | `geometry_msgs/Pose` | `gz.msgs.Pose` |
<!-- 위 표: 브리지되는 토픽들. ROS에서 Gazebo로 가는 것은 <ns>/cmd_vel 하나뿐이고 나머지는 전부 시뮬레이터가 내보내는 방향. -->

The file spells the namespace out per robot. It used to carry a full `/robot1` set alongside the `/robot2` set even though only `robot2` is spawned; that block is now commented out, because a live-looking config for a robot that does not exist is what makes readers ask why the single robot is numbered two. Restoring it means adding the entry back to `robot_info` and uncommenting the block.
<!-- 파일에는 네임스페이스가 로봇별로 적혀 있음. 예전에는 robot2 만 스폰되는데도 /robot1 한 벌이 /robot2 한 벌과 함께 살아 있었는데, 지금은 그 블록을 주석 처리했음 — 존재하지 않는 로봇의 설정이 살아 있는 것처럼 보이는 게 "로봇이 한 대인데 왜 2번이냐"는 의문의 원인이기 때문. 되살리려면 robot_info 에 항목을 다시 넣고 그 블록의 주석만 풀면 됨. -->

`/robot2/pose_gt` is the exception to the namespace pattern: its Gazebo side is `/model/mecanum_lift_robot/pose`, keyed by model name rather than namespace, because that is what the pose-publisher system emits. `waypoint_follower` subscribes to it instead of `odom` because wheel odometry drifts under slip.
<!-- /robot2/pose_gt 는 네임스페이스 규칙의 예외임. Gazebo 쪽 이름이 /model/mecanum_lift_robot/pose 로 네임스페이스가 아니라 모델 이름 기준인데, pose-publisher 시스템이 그렇게 내보내기 때문. waypoint_follower 는 바퀴 오도메트리가 슬립으로 어긋나기 때문에 odom 대신 이 토픽을 구독함. -->

`/clock` deliberately carries no namespace — it is world-wide, not per-robot. The simulated camera publishes to `<ns>/camera/image` inside Gazebo but is not bridged, because the gesture pipeline uses a real webcam instead.
<!-- /clock 은 일부러 네임스페이스를 안 붙임 — 로봇별이 아니라 월드 전역이기 때문. 시뮬레이션 카메라는 Gazebo 안에서 <ns>/camera/image 로 발행하지만 브리지하지 않음 — 수신호 파이프라인이 실물 웹캠을 쓰기 때문. -->

`tf` is the other topic with no namespace, and unlike `/clock` that is not a choice this repo made. `tf2_ros` hard-codes the topic names as absolute paths — `"/tf"` and `"/tf_static"` with a leading slash, in `transform_broadcaster.hpp` and `transform_listener.hpp` — so a node's namespace never applies to them. The namespaced `robot_state_publisher` has always published its link transforms to the global `/tf`; only the bridged Gazebo side was named `<ns>/tf`, and nothing subscribed to it, which left `odom → base_link` missing from the tf tree every tool actually reads.
<!-- tf 도 네임스페이스가 없는 토픽인데, /clock 과 달리 이건 이 리포지토리가 고른 게 아님. tf2_ros 가 토픽 이름을 절대 경로로 박아 놓았음 — transform_broadcaster.hpp 와 transform_listener.hpp 안의 앞 슬래시 붙은 "/tf", "/tf_static" — 그래서 노드에 네임스페이스를 줘도 tf 에는 적용되지 않음. 네임스페이스가 붙은 robot_state_publisher 도 링크 변환을 처음부터 전역 /tf 로 발행해 왔고, 브리지된 Gazebo 쪽만 <ns>/tf 라는 이름이었는데 그걸 구독하는 게 아무도 없었음. 그 결과 도구들이 실제로 읽는 tf 트리에서 odom → base_link 가 빠져 있었음. -->

Robots are therefore separated in tf by **frame name**, not by topic. `robot_state_publisher` gets `frame_prefix: <ns>/`, and the URDF spells the same prefix into the `DiffDrive` plugin's `frame_id` / `child_frame_id` and into every sensor's `gz_frame_id`, so the frames are `robot2/odom`, `robot2/base_link`, `robot2/lidar_link`, and so on. Both halves must use the identical string or the tree breaks in two at `base_link`.
<!-- 그래서 tf 에서 로봇은 토픽이 아니라 프레임 이름으로 갈림. robot_state_publisher 에 frame_prefix: <ns>/ 를 주고, URDF 는 DiffDrive 플러그인의 frame_id·child_frame_id 와 모든 센서의 gz_frame_id 에 같은 접두어를 적음. 그래서 프레임 이름이 robot2/odom, robot2/base_link, robot2/lidar_link 같은 형태가 됨. 두 쪽이 완전히 같은 문자열을 써야 하고, 다르면 트리가 base_link 에서 두 조각으로 끊어짐. -->

Velocity sources arbitrated by `twist_mux`, highest priority first:
<!-- twist_mux 가 중재하는 속도 명령 소스, 우선순위가 높은 순: -->

| Input topic | Priority | Timeout | Publisher today |
|---|---|---|---|
| `cmd_vel_estop` | 100 | 0.3 s | none yet |
| `cmd_vel_gesture` | 50 | 0.5 s | that robot's own `camera_node`, in the same namespace |
| `cmd_vel_teleop` | 20 | 0.5 s | `teleop_twist_keyboard`, for manual checks |
| `cmd_vel_auto` | 10 | 0.5 s | `mission_follower` when `enable_patrol` is on (only while driving a dispatch route) |
<!-- 위 표: twist_mux 입력 4개의 우선순위·타임아웃·현재 발행자. estop 은 아직 발행자가 없음. -->

Because `twist_mux` owns the output, nothing else may publish to `<ns>/cmd_vel` directly — manual driving goes into `cmd_vel_teleop`:
<!-- twist_mux 가 출력을 소유하므로 다른 것이 <ns>/cmd_vel 에 직접 발행하면 안 됨 — 수동 주행은 cmd_vel_teleop 로 넣을 것: -->

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/robot2/cmd_vel_teleop
```
<!-- 위 명령: teleop_twist_keyboard 의 /cmd_vel 을 /robot2/cmd_vel_teleop 로 리맵해 실행. -->

Driving by hand while `enable_patrol` is on means fighting the follower for the mux, since teleop only outranks it — pass `enable_patrol:=false` for a clean manual check.
<!-- enable_patrol 이 켜진 채로 수동 주행을 하면 mux 를 두고 팔로워와 겨루게 됨. teleop 이 우선순위만 높을 뿐이기 때문 — 깔끔하게 수동 확인을 하려면 enable_patrol:=false 를 줄 것. -->

`<ns>/scan_rays` (`visualization_msgs/MarkerArray`) is not bridged either — `scan_rays` builds it inside ROS from the scan that was already bridged, so it is a second view of existing data rather than a new sensor.
<!-- <ns>/scan_rays(visualization_msgs/MarkerArray) 도 브리지 대상이 아님 — scan_rays 가 이미 브리지된 스캔을 ROS 안에서 다시 그린 것이라, 새 센서가 아니라 있는 데이터를 한 번 더 보는 것임. -->

`camera_node` itself publishes `<ns>/gesture` (`std_msgs/String`, the recognized label, for logging and latency measurement), `<ns>/cmd_vel_gesture` (`geometry_msgs/Twist`, motion labels only), `<ns>/signal_dispatch` (`std_msgs/String`, the destination zone for dispatch labels, read by `mission_follower`), and `<ns>/image_webcam` (`sensor_msgs/Image`, off by default). These carry the robot namespace because `bringup` starts the node inside it; started bare with `ros2 run` the same topics appear at the root instead.
<!-- camera_node 자체가 발행하는 것: <ns>/gesture(std_msgs/String — 인식된 라벨, 기록·지연시간 측정용), <ns>/cmd_vel_gesture(geometry_msgs/Twist, 속도 라벨만), <ns>/signal_dispatch(std_msgs/String — 파견 라벨의 목적지 존 이름, mission_follower 가 읽음), <ns>/image_webcam(sensor_msgs/Image, 기본 꺼짐). bringup 이 이 노드를 로봇 네임스페이스 안에서 띄우기 때문에 네임스페이스가 붙는 것이며, ros2 run 으로 그냥 띄우면 같은 토픽들이 루트에 생김. -->

## Build & run

<!-- 빌드 및 실행 -->

From the repo root:
<!-- 리포지토리 루트에서: -->

```bash
colcon build --symlink-install          # also picks up examples/src practice packages
colcon build --packages-select robot_control signal_vision auto_drive knavi_bringup
source install/setup.bash
```
<!-- 위 명령: 루트에서 colcon build 를 하면 examples/src 의 연습용 패키지까지 함께 빌드됨. 이 프로젝트 패키지만 빌드하려면 --packages-select 에 네 개를 나열하고, 이후 install/setup.bash 를 source 함. -->

Moving or renaming a file that `data_files` installs leaves a dangling symlink in `build/` and `install/` under `--symlink-install`, and the next build fails with `error: can't copy ...: doesn't exist`. Delete that package's stale directory under `build/` and `install/` and build again.
<!-- --symlink-install 상태에서 data_files 로 설치되는 파일을 옮기거나 이름을 바꾸면 build/ 와 install/ 에 깨진 심볼릭 링크가 남고, 다음 빌드가 error: can't copy ...: doesn't exist 로 실패함. 그 패키지의 낡은 디렉터리를 build/ 와 install/ 에서 지우고 다시 빌드할 것. -->

Launching:
<!-- 실행: -->

```bash
ros2 launch knavi_bringup bringup.launch.py
ros2 launch knavi_bringup bringup.launch.py headless:=true enable_camera:=false
ros2 launch knavi_bringup bringup.launch.py camera_device_id:=1
ros2 launch knavi_bringup bringup.launch.py enable_rviz:=false
ros2 run signal_vision camera_node --ros-args -p enable_inference:=false
```
<!-- 위 명령: bringup 기본 실행, GUI·카메라 없이 실행, 웹캠 장치 번호 지정 실행, RViz 창 없이 실행, 그리고 camera_node 단독 실행(추론 끔). -->

Launch arguments: `world` (defaults to `navi_factory.sdf`, path computed from the workspace root), `use_sim_time` (`true`), `headless` (`false`), `enable_camera` (`true`), `camera_device_id` (empty), `enable_patrol` (`true`), `enable_flat_ground` (`true`), `enable_rviz` (`true`).
<!-- launch 인자: world(기본값 navi_factory.sdf, 경로는 워크스페이스 루트 기준 자동 계산), use_sim_time(true), headless(false), enable_camera(true), camera_device_id(비어 있음), enable_patrol(true), enable_flat_ground(true), enable_rviz(true). -->

`enable_rviz` is on by default, so a plain `bringup` opens RViz2 on `config/knavi.rviz` next to the `gz sim` window: the lidar scan as points, the same scan again as ray line segments, the tf axes, fixed frame `robot2/odom`. It is part of bringup rather than a separate step because the `gz sim` window only draws the sensor's own rays and cannot tell you whether the scan reached ROS at all.
<!-- enable_rviz 는 기본이 켜짐이라, 그냥 bringup 하면 gz sim 창 옆에 config/knavi.rviz 로 RViz2 가 함께 뜸 — 라이다 스캔은 점으로, 같은 스캔을 광선 선분으로 한 번 더, tf 는 좌표축으로, 고정 프레임은 robot2/odom. 별도 단계가 아니라 bringup 에 넣은 이유는, gz sim 창은 센서 자신의 광선만 그릴 뿐 그 스캔이 ROS 까지 넘어왔는지는 알려 주지 못하기 때문. -->

The ray line segments come from `scan_rays`, which starts and stops with RViz because it is a viewer aid and nothing else reads it. RViz's `LaserScan` display puts a point only where a return came back, so a ray that hit nothing within range draws nothing at all — measured on the 180° scan, 283 of 360 rays were `inf`, leaving most of the fan blank and no way to tell "the sensor does not look there" from "it looked and the space is empty". `scan_rays` republishes the same scan on `<ns>/scan_rays` as two `Marker` line lists, returns in red and misses drawn out to range in cyan. It never rewrites `<ns>/scan` itself: turning `inf` into a range value there would read as a wall at 10 m to the estop node and to Nav2 later.
<!-- 광선 선분은 scan_rays 가 그리는 것이고, 뷰어 보조용이라 읽는 쪽이 RViz 뿐이어서 RViz 와 함께 뜨고 함께 꺼짐. RViz 의 LaserScan 디스플레이는 반사가 돌아온 자리에만 점을 찍으므로, 사거리 안에서 아무것도 못 맞힌 광선은 아예 안 그려짐 — 180도 스캔에서 실측하니 360개 중 283개가 inf 였고, 부채꼴 대부분이 빈 채로 남아서 "센서가 저쪽을 안 본다"와 "봤는데 비어 있다"를 구분할 수 없었음. scan_rays 는 같은 스캔을 <ns>/scan_rays 에 Marker 선분 목록 두 개로 다시 발행함 — 반사가 온 광선은 빨강, 못 맞힌 광선은 사거리 끝까지 청록. <ns>/scan 자체는 절대 고치지 않음: 거기서 inf 를 거리 값으로 바꾸면 estop 노드와 나중의 Nav2 가 10m 앞에 벽이 있다고 읽게 됨. -->

Two ways it stays shut: `enable_rviz:=false`, or `headless:=true`, which wins over `enable_rviz` because a run asked to have no GUI must not open a window.
<!-- 안 뜨게 하는 방법은 둘: enable_rviz:=false, 또는 headless:=true. headless 가 enable_rviz 를 이기는데, GUI 없이 돌리라고 시킨 실행이 창을 띄우면 안 되기 때문. -->

That config carries no `RobotModel` display: the only link with a visual is `chassis`, and its mesh URI is `model://mecanum_lift/...`, which RViz's resource retriever cannot resolve — it handles `package://`, `file://` and `http://` only.
<!-- 그 설정에는 RobotModel 디스플레이가 없음 — visual 을 가진 링크가 chassis 하나뿐인데 그 메시 URI 가 model://mecanum_lift/... 이고, RViz 의 resource retriever 는 package://, file://, http:// 만 풀 수 있어서 이걸 못 읽기 때문. -->

`enable_flat_ground` must stay on in the merged world: the current `warehouse` model carries no `<collision>` at all, so with the plate off the robot falls through the floor forever (measured: z below −3000 within seconds). The old advice to turn it off dated from the previous world, whose defective STL floor still existed as a collision.
<!-- enable_flat_ground 는 병합된 월드에서 반드시 켜 두어야 함. 지금 warehouse 모델에는 <collision> 이 하나도 없어서, 판을 끄면 로봇이 바닥을 뚫고 무한 낙하함(실측: 몇 초 만에 z가 -3000 아래). 끄라던 옛 안내는 결함 STL 바닥이나마 콜리전으로 존재하던 이전 월드 기준이었음. -->

`ls /dev/video*` shows which webcam index exists; re-plugging a USB camera renumbers it.
<!-- ls /dev/video* 로 실제 존재하는 웹캠 번호를 확인할 것. USB를 다시 꽂으면 번호가 다시 매겨짐. -->

Shell entry points in `scripts/`, each also bound to a VS Code task:
<!-- scripts/ 안의 실행 스크립트들, 각각 VS Code 태스크에도 연결되어 있음: -->

- `build_workspace.sh` — installs ROS 2 Jazzy if missing, then builds
  <!-- build_workspace.sh — ROS 2 Jazzy가 없으면 설치한 뒤 빌드 -->
- `run_gazebo.sh` — the world alone, no ROS side
  <!-- run_gazebo.sh — ROS 쪽 없이 월드만 실행 -->
- `run_full_stack.sh` — `rosbridge_server` plus `gz sim`, for driving the sim from the Signal-Vision repo over the rosbridge websocket
  <!-- run_full_stack.sh — rosbridge_server와 gz sim을 함께 실행. Signal-Vision 리포지토리가 rosbridge 웹소켓으로 시뮬레이션을 조종하기 위한 구성 -->
- `run_bringup.sh` — the full robot bringup
  <!-- run_bringup.sh — 로봇 전체 bringup 실행 -->
- `run_camera_node.sh` — `camera_node` alone, as a webcam-to-gesture unit check
  <!-- run_camera_node.sh — camera_node 만 단독 실행, 웹캠→수신호 경로 단위 점검용 -->
- `setup_infer_env.py` — shallow-clones Signal-Vision at its latest tag, vendors that source into `signal_vision/vision_hand/`, and installs the runtime libraries into system python3.12. It builds no venv.
  <!-- setup_infer_env.py — Signal-Vision 을 최신 태그로 얕게 clone 해서 그 소스를 signal_vision/vision_hand/ 로 벤더 복사하고, 런타임 라이브러리를 시스템 python3.12 에 설치함. venv 는 만들지 않음. -->
- `setup_perception_env.py`, `setup_repo_venv.py` — build `.venv-perception` for the Signal-transport-perception repository; `setup_repo_venv.py` holds the shared venv logic
  <!-- setup_perception_env.py, setup_repo_venv.py — Signal-transport-perception 리포지토리용 .venv-perception 을 만듦. setup_repo_venv.py 는 공용 venv 로직을 담고 있음 -->

A venv cannot serve `ros2 run`: colcon writes console scripts with a `/usr/bin/python3` shebang no matter which environment is active, so the libraries a node imports have to be in system python3.12. That is why `setup_infer_env.py` installs there and why `run_camera_node.sh` activates nothing.
<!-- venv 로는 ros2 run 을 지원할 수 없음. colcon 이 어떤 환경이 활성화돼 있든 console script 의 shebang 을 /usr/bin/python3 로 쓰기 때문에, 노드가 import 하는 라이브러리는 시스템 python3.12 에 있어야 함. setup_infer_env.py 가 거기에 설치하는 이유이자 run_camera_node.sh 가 아무것도 활성화하지 않는 이유임. -->

Tests are style checks only, run with `colcon test --packages-select robot_control signal_vision auto_drive knavi_bringup`.
<!-- 테스트는 스타일 검사뿐이며, colcon test --packages-select robot_control signal_vision auto_drive knavi_bringup 로 실행함. -->

GUI programs (`gz sim` with rendering, RViz) must be started from the user's own desktop terminal — launching them from an agent shell fails for lack of a GL context.
<!-- GUI 프로그램(렌더링을 켠 gz sim, RViz)은 사용자가 자기 데스크톱 터미널에서 직접 실행해야 함 — 에이전트 셸에서 띄우면 GL 컨텍스트가 없어 실패함. -->

## Language & conventions

<!-- 언어 및 코딩 컨벤션 -->

Python 3.12. Naming:
<!-- Python 3.12. 명명 규칙: -->

- Classes: `PascalCase` (e.g. `RobotController`)
  <!-- 클래스명: PascalCase (대문자로 시작하는 카멜 표기) -->
- Functions: `snake_case` (e.g. `publish_image`)
  <!-- 함수명: snake_case (소문자로 시작하는 스네이크 표기) -->

Source comments and docstrings inside `src/` are written in Korean and explain **why**, not what. The English-body plus Korean-comment rule at the top of this file applies to markdown docs, not to code comments.
<!-- src/ 안의 주석과 독스트링은 한국어로 쓰고, 무엇을 하는지가 아니라 왜 그렇게 했는지를 설명함. 이 파일 맨 위의 영어 본문 + 한글 주석 규칙은 마크다운 문서에 적용되는 것이지 코드 주석에는 적용되지 않음. -->

`ament_flake8` and `ament_pep257` run as tests, so import order and docstring style are enforced — keep new code passing them.
<!-- ament_flake8 과 ament_pep257 이 테스트로 돌기 때문에 import 순서와 독스트링 스타일이 강제됨 — 새 코드도 이를 통과하게 유지할 것. -->
