# design.md — Project overview & current status

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. It covers **what this project is and where it currently stands**. For the concrete code structure (packages, files, node/topic graph, build/run commands, conventions) see `doc/AGENT.md`.

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

## Repository purpose

<!-- 이 리포지토리가 어떤 용도인지 -->

Signal-Simulation is the Gazebo simulation + robot side of a larger project. Its scope:
<!-- Signal-Simulation은 더 큰 프로젝트의 Gazebo 시뮬레이션 + 로봇 담당 부분임. 범위: -->

- Gazebo world and robot model setup (worlds, SDF/URDF models, launch configuration)
  <!-- Gazebo 월드 및 로봇 모델(SDF/URDF) 구성, launch 설정 -->
- ROS 2 integration for the simulated robot
  <!-- 시뮬레이션 로봇에 대한 ROS 2 연동 -->
- Design of how simulation, robot control, and ROS 2 topics/nodes fit together
  <!-- 시뮬레이션 / 로봇 제어 / ROS 2 토픽·노드가 어떻게 맞물리는지에 대한 설계 -->

## Ownership boundary with the other repositories

<!-- 다른 리포지토리와의 소유 경계 -->

Camera-based image processing and gesture recognition are **developed in the Signal-Vision repository**, not here. Do not author or modify image-processing or model code in this repo.
<!-- 카메라 기반 이미지 프로세싱과 수신호 인식은 Signal-Vision 리포지토리에서 개발됨 — 여기가 아님. 이 리포지토리에서 이미지 프로세싱·모델 코드를 새로 쓰거나 고치지 말 것. -->

That said, a copy of Signal-Vision's source **does** live here, at `src/signal_vision/signal_vision/vision_hand/`. It was pulled in because the simulation has to run the inference in-process: `camera_node` imports it directly instead of talking to a second process. It is a vendored copy, not a fork:
<!-- 다만 Signal-Vision 소스의 복사본이 실제로 여기에 있음 — src/signal_vision/signal_vision/vision_hand/. 시뮬레이션이 추론을 같은 프로세스 안에서 돌려야 해서 긁어온 것으로, camera_node가 별도 프로세스와 통신하지 않고 이 코드를 직접 import함. 포크가 아니라 벤더 사본임: -->

- `scripts/setup_infer_env.py` shallow-clones the Signal-Vision repository at its latest tag and copies that source into `vision_hand/`, rewriting its internal `src.*` imports so the copy is self-contained.
  <!-- scripts/setup_infer_env.py 가 Signal-Vision 리포지토리를 최신 태그로 얕게 clone 해서 그 소스를 vision_hand/ 로 복사하고, 사본이 자기 완결적으로 동작하도록 내부의 src.* import 를 고쳐 씀. -->
- It used to build a `.venv-infer` and copy the pip-installed package out of it, but that venv was deleted as soon as the copy finished — it pulled a whole dependency tree down just to throw it away, and it could never have been the runtime environment anyway.
  <!-- 예전에는 .venv-infer 를 만들어 거기 pip 설치된 패키지를 복사해 왔지만, 그 venv 는 복사가 끝나자마자 지워졌음 — 의존성 트리를 통째로 받아 놓고 버리는 셈이었고, 애초에 실행 환경이 될 수도 없었음. -->
- Changes to recognition behavior belong upstream in Signal-Vision; refresh the copy by re-running that script rather than editing files under `vision_hand/`.
  <!-- 인식 동작을 바꾸는 작업은 상류의 Signal-Vision 몫임. vision_hand/ 아래 파일을 직접 고치지 말고, 그 스크립트를 다시 실행해 사본을 갱신할 것. -->
- `scripts/setup_perception_env.py` does the same for a second external repository, Signal-transport-perception, in a separate `.venv-perception` — separate because the two repositories' dependencies conflict.
  <!-- scripts/setup_perception_env.py 는 또 다른 외부 리포지토리인 Signal-transport-perception에 대해 같은 일을 하되 별도의 .venv-perception 을 씀 — 두 리포지토리의 의존성이 충돌하기 때문. -->

What this repo owns is everything downstream of the recognized label: turning a label into a velocity command, arbitrating command sources, and driving the robot.
<!-- 이 리포지토리가 소유하는 것은 인식된 라벨 이후의 모든 것 — 라벨을 속도 명령으로 바꾸고, 명령 소스들을 중재하고, 로봇을 구동하는 부분. -->

The package named `signal_vision` is this repo's own, despite the name overlap: `camera_node` and `metrics.py` under it are written here, and only `vision_hand/` inside it is the copy.
<!-- signal_vision 이라는 패키지는 이름이 겹칠 뿐 이 리포지토리의 것임. 그 아래 camera_node 와 metrics.py 는 여기서 작성한 코드이고, 그 안의 vision_hand/ 만 복사본임. -->

## Why this project exists

<!-- 이 프로젝트의 목적 -->

**Phase 1 (core goal):** A camera detects a signal worker — the person directing traffic in a factory — and their hand gestures. The frames go through OpenCV and a landmark extractor into an AI model that outputs a discrete command (stop / forward / turn left / turn right). This repo turns that command into a velocity and drives a simulated cargo-carrying mobility robot in Gazebo. Motivation: prevent fatal accidents from workers not seeing a signal worker's directions, and let a human command the robot directly by hand signal instead of a separate controller.
<!--
1단계(핵심 목표): 카메라가 신호수와 그 손동작을 감지 → OpenCV와 랜드마크 추출기를 거쳐
AI 모델이 정지/전진/좌회전/우회전 같은 이산적인 명령을 출력 →
이 리포지토리가 그 명령을 속도로 바꿔 Gazebo 안의 짐 나르는 모빌리티 로봇을 움직임.
동기: 신호수의 지시를 못 본 작업자로 인한 사망사고 예방, 그리고 별도 컨트롤러 없이
사람이 수신호로 로봇을 직접 명령할 수 있게 하기 위함.
-->

**Phase 2 (stretch goal, only if time permits):** The robot also does obstacle detection and re-plans its path autonomously to reach a destination within the factory, layered on top of (not replacing) the manual signal-based control. The `cmd_vel_auto` input of `twist_mux` is the reserved slot for it, at the lowest priority — a human signal always wins over autonomy.
<!--
2단계(시간 남으면): 로봇이 장애물 탐지 + 경로 재탐색으로 공장 내 목적지까지 자율 주행.
수신호 기반 수동 제어를 대체하는 게 아니라 그 위에 얹는 추가 기능임.
twist_mux 의 cmd_vel_auto 입력이 그 자리로 예약되어 있고 우선순위가 가장 낮음 —
사람의 수신호가 언제나 자율주행을 이김.
-->

## Current status

<!-- 현재 상태 -->

The Phase 1 driving skeleton exists and runs. `ros2 launch knavi_bringup bringup.launch.py` brings up the world, spawns the robot, and the robot drives — from the keyboard, from recognized hand signals, and from the track follower. Unlike the earlier draft state of this repo, this has been built and run on the Linux VM, not just written.
<!-- 1단계 구동 뼈대가 존재하고 실제로 돎. ros2 launch knavi_bringup bringup.launch.py 로 월드가 뜨고 로봇이 스폰되며, 키보드로도 인식된 수신호로도 트랙 팔로워로도 로봇이 주행함. 예전 초안 상태와 달리 이제는 리눅스 VM에서 빌드·실행까지 확인된 상태임. -->

The code was one package, `robot_control`, and is now four: `robot_control` keeps the urdf and the config, `signal_vision` took the camera and gesture code, `auto_drive` took the driving nodes, and `knavi_bringup` holds the launch that assembles them. The split is by responsibility, so each side of the project can be worked on without touching the others.
<!-- 코드는 원래 robot_control 한 패키지였고 지금은 네 개임. robot_control 은 urdf 와 config 를 유지하고, signal_vision 이 카메라·수신호 코드를, auto_drive 가 주행 노드를 가져갔으며, knavi_bringup 이 그것들을 조립하는 launch 를 가짐. 책임 단위로 나눈 것이라 프로젝트의 각 부분을 서로 건드리지 않고 작업할 수 있음. -->

The command path today:
<!-- 현재의 명령 경로: -->

```
webcam → camera_node (capture → inference → label)
                  → cmd_vel_gesture ─┐
teleop            → cmd_vel_teleop ──┤
waypoint_follower → cmd_vel_auto ────┼→ twist_mux → /robot2/cmd_vel → ros_gz_bridge → drive plugin
(estop: reserved) ───────────────────┘
```
<!-- 위 그림: 웹캠 → camera_node(캡처→추론→라벨) → cmd_vel_gesture, teleop → cmd_vel_teleop, waypoint_follower → cmd_vel_auto 가 twist_mux 로 모여 /robot2/cmd_vel 로 나가고, 브릿지를 거쳐 구동 플러그인에 도달함. estop 입력은 예약된 자리. -->

Two ways the vision side can reach the simulation, both in use:
<!-- 비전 쪽이 시뮬레이션에 닿는 경로가 두 가지 있고, 둘 다 쓰이고 있음: -->

- **In-process (default).** `camera_node` runs the vendored model itself and publishes `cmd_vel_gesture`. Capture and inference share one worker thread because inference is the bottleneck, and rendering runs on a second thread so GUI updates do not cut throughput.
  <!-- 같은 프로세스 방식(기본값). camera_node 가 벤더링된 모델을 직접 돌리고 cmd_vel_gesture 를 발행함. 추론이 병목이라 캡처와 추론이 워커 스레드 하나를 공유하고, GUI 갱신이 처리율을 깎지 않도록 렌더링만 두 번째 스레드로 분리함. -->
- **Over rosbridge.** `scripts/run_full_stack.sh` runs `rosbridge_server` next to `gz sim` so the Signal-Vision repository can send commands over a websocket from outside the ROS 2 graph.
  <!-- rosbridge 방식. scripts/run_full_stack.sh 가 gz sim 옆에 rosbridge_server 를 함께 띄워, Signal-Vision 리포지토리가 ROS 2 그래프 밖에서 웹소켓으로 명령을 보낼 수 있게 함. -->

The world is `worlds/navi_factory`, built from the AWS RoboMaker warehouse models plus project-specific props. Its floor collision mesh is defective — the horizontal floor triangles have their normals flipped downward, and the mesh contains degenerate zero-area triangles — so a robot spawned on it fell through and toppled. The workaround is `config/flat_ground.sdf`, a collision-only plate spawned at z = 0.25 that the robot rests on; the warehouse mesh is kept for wall collisions.
<!-- 월드는 worlds/navi_factory 이며, AWS RoboMaker 창고 모델에 프로젝트 전용 소품을 더해 만든 것임. 이 월드의 바닥 콜리전 메시가 결함이 있음 — 수평 바닥 삼각형들의 법선이 전부 아래로 뒤집혀 있고, 면적 0인 퇴화 삼각형도 섞여 있음 — 그래서 그 위에 스폰한 로봇이 파고들며 넘어졌음. 우회책으로 config/flat_ground.sdf(콜리전 전용 평면)를 z = 0.25 에 스폰해 로봇이 그 위에 서게 했고, 창고 메시는 벽 충돌용으로 그대로 둠. -->

## Open items

<!-- 미해결 항목 -->

- **The command interface with the vision repository is still not agreed.** The label set used here (`STOP`, `FORWARD`, `LEFT`, `RIGHT`) and the topic names are this repo's own draft. Topic naming, the command set, and whether a confidence value travels with the command all need to be settled with the other side.
  <!-- 비전 리포지토리와의 명령 인터페이스가 아직 합의되지 않음. 여기서 쓰는 라벨 집합(STOP, FORWARD, LEFT, RIGHT)과 토픽 이름은 이쪽에서 혼자 만든 초안임. 토픽 이름, 명령 종류, 신뢰도 값을 명령과 함께 보낼지 여부를 상대 쪽과 확정해야 함. -->
- **No hold policy for a dropped label.** `twist_mux` drops the gesture source after 0.5 s without a message, so a brief recognition gap stops a moving robot. Holding the last label would smooth this, but the safe policy is probably asymmetric — hold `STOP` long, release `FORWARD` quickly — and that is a behavior decision, not a refactor.
  <!-- 라벨이 끊겼을 때의 유지 정책이 없음. twist_mux 는 0.5초 동안 메시지가 없으면 gesture 소스를 버리므로, 인식이 잠깐만 끊겨도 주행 중이던 로봇이 멈춤. 마지막 라벨을 유지하면 부드러워지지만, 안전한 정책은 아마 비대칭일 것임 — STOP은 오래 유지하고 FORWARD는 빨리 놓는 식 — 그리고 이건 리팩터링이 아니라 동작에 대한 결정임. -->
- **Emergency stop has no publisher.** `cmd_vel_estop` is the highest-priority input but nothing publishes to it yet.
  <!-- 비상정지에 발행자가 없음. cmd_vel_estop 이 최우선 입력이지만 아직 아무도 여기에 발행하지 않음. -->
- **Phase 2 navigation is only wired up to the sensor.** A 2D `gpu_lidar` is bridged as `<ns>/scan`. `waypoint_follower` drives a fixed track from ground-truth pose, which is not navigation: Nav2, SLAM, and the costmap configuration are not present yet, and nothing reads the scan. A 2D scan sees one horizontal slice, so low pallets and overhanging objects stay invisible until a depth sensor is added.
  <!-- 2단계 내비게이션은 센서까지만 연결되어 있음. 2D gpu_lidar 를 <ns>/scan 으로 브리지했음. waypoint_follower 는 ground-truth 좌표를 보고 고정된 트랙을 따라갈 뿐 내비게이션이 아님. Nav2·SLAM·코스트맵 설정은 아직 없고, 스캔을 읽는 것도 아직 없음. 2D 스캔은 수평 단면 한 장만 보므로 낮은 팔레트나 머리 위로 튀어나온 물체는 깊이 센서를 추가하기 전까지 보이지 않음. -->
- **The lidar only sees forward, and that is the mount, not the sensor.** It used to sit at `(0.3, 0, 1.1)` on the chassis, which is inside the `mecanum_lift` visual mesh — every one of the 360 rays came back between 0.32 m and 1.43 m and no warehouse wall was ever visible. `gpu_lidar` renders visuals rather than collisions, so clearing the URDF collision box did nothing, and the mesh keeps its `x[-1.07, 0.99]` footprint at every height from 0 to 2.01 m, leaving no clear height inside the body. It now sits on the front bumper at `(1.25, 0, 0.35)` and reaches out past 9.9 m, but at 360° the body still blocked the rear: 148 of the 360 rays (41%) hit the robot itself, and the clear arc measured in simulation is −105.8° to +105.8°.
  <!-- 라이다가 앞쪽만 보는데, 그건 센서가 아니라 장착 위치 때문임. 예전에는 chassis 기준 (0.3, 0, 1.1) 이었고 그 자리는 mecanum_lift 외형 메시 안쪽이었음 — 광선 360개가 전부 0.32~1.43m 에서 돌아왔고 창고 벽은 한 번도 보이지 않았음. gpu_lidar 는 collision 이 아니라 visual 을 렌더링해서 맞히므로 URDF collision 박스를 넘긴 것만으로는 소용이 없었고, 그 메시는 높이 0~2.01m 전 구간에서 x[-1.07, 0.99] footprint 를 유지해서 몸통 안에는 트이는 높이가 없음. 지금은 앞 범퍼 (1.25, 0, 0.35) 에 달려 9.9m 넘게 보지만, 360도로 두면 뒤쪽은 여전히 몸통에 막혔음: 광선 360개 중 148개(41%)가 로봇 자신에 맞았고, 시뮬레이션에서 실측한 트인 구간은 -105.8도 ~ +105.8도임. -->
- **The lidar's field of view is now the front 180°, and the rear is given up on purpose.** `min_angle` / `max_angle` are `±1.5708`, which fits inside the measured clear arc, so all 360 samples land on ground the sensor can actually see: measured in simulation, self-hit rays went from 148 of 360 to 0, and the angular step from 1° to 0.5°. The cost is that the robot is blind behind, accepted because it is not planned to reverse; Nav2 and SLAM would rather have the rear, and the answer there is a second lidar on the rear bumper, not widening this one back to 360° (which only ever returned the robot's own body).
  <!-- 라이다 시야각은 이제 전방 180도이고, 뒤쪽은 의도적으로 포기했음. min_angle·max_angle 이 ±1.5708 이고 이 값은 실측한 트인 구간 안에 들어가므로, 샘플 360개가 전부 실제로 볼 수 있는 곳에 떨어짐: 시뮬레이션 실측으로 자기 몸에 맞는 광선이 360개 중 148개에서 0개가 됐고, 각 간격은 1도에서 0.5도가 됐음. 대가는 뒤가 안 보인다는 것이고, 후진할 계획이 없어서 받아들인 것임. Nav2·SLAM 은 뒤쪽을 원하는데, 그때 답은 뒤 범퍼에 라이다를 한 대 더 다는 것이지 이 각을 360도로 되돌리는 게 아님(되돌려 봐야 로봇 자기 몸만 돌아왔음). -->
- **A second robot needs a `map` root frame first — decided, not built.** Robots are separated in tf by frame name inside the single global `/tf`, which works, but it leaves each robot as its own disconnected fragment: `robot1/odom → robot1/base_link` and `robot2/odom → robot2/base_link` share no parent. RViz has exactly one fixed frame, so `config/knavi.rviz` fixed at `robot2/odom` would draw nothing for the second robot except `No transform from [robot1/base_link] to [robot2/odom]`. The decision is a `map` root: one `map → <ns>/odom` transform per robot, published from inside the launch's robot loop using the spawn pose already sitting in `robot_info`, with the RViz fixed frame moved to `map`. SLAM replaces those static transforms per robot when it lands, so this is the frame layout Nav2 wants anyway. Not built yet because one robot needs none of it.
  <!-- 두 번째 로봇을 붙이려면 map 루트 프레임이 먼저 필요함 — 결정은 났고 아직 만들지 않았음. 로봇은 전역 /tf 하나 안에서 프레임 이름으로 갈리고 그건 잘 동작하지만, 그 결과 로봇마다 서로 끊긴 조각으로 남음: robot1/odom → robot1/base_link 와 robot2/odom → robot2/base_link 에 공통 부모가 없음. RViz 는 fixed frame 을 딱 하나만 가지므로, config/knavi.rviz 를 robot2/odom 으로 고정해 두면 두 번째 로봇 쪽은 "No transform from [robot1/base_link] to [robot2/odom]" 만 뜨고 아무것도 안 그려짐. 결정은 map 루트를 두는 것: 로봇마다 map → <ns>/odom 을 하나씩, launch 의 로봇 루프 안에서 robot_info 에 이미 들어 있는 스폰 pose 로 발행하고, RViz 의 fixed frame 을 map 으로 옮김. 나중에 SLAM 이 들어오면 그 정적 변환들을 로봇별로 대체하므로, 어차피 Nav2 가 원하는 프레임 구성임. 로봇이 한 대면 이 중 아무것도 필요 없어서 아직 만들지 않았음. -->
- **The robot model is a placeholder.** `robot.urdf.xacro` describes a temporary diff-drive body (20 kg, 0.135 m wheels, 0.89 m track) wearing an OGV mesh. Replace it when the real vehicle specification arrives.
  <!-- 로봇 모델은 임시임. robot.urdf.xacro 는 OGV 메시를 씌운 임시 차동구동 차체(20 kg, 바퀴 반지름 0.135 m, 좌우 간격 0.89 m)를 기술한 것임. 실제 차량 스펙이 나오면 교체할 것. -->
- **Cameras are paired to robots by `/dev/video<N>`, which is not a stable name.** One robot gets one camera, wired through the robot's namespace, and the pairing lives in `robot_info`. The kernel renumbers those device nodes on replug, so with two cameras the pairing can silently swap between boots and a signal worker would be driving the wrong robot. A stable identifier such as a `/dev/v4l/by-id/` path fixes it, but `camera_node` takes an integer today, so this is a parameter change and not just configuration.
  <!-- 카메라와 로봇이 /dev/video<N> 으로 짝지어져 있는데 그건 안정적인 이름이 아님. 로봇 한 대에 카메라 한 대이고 로봇의 네임스페이스로 배선되며, 그 짝은 robot_info 에 있음. 커널이 다시 꽂을 때 장치 번호를 다시 매기므로, 카메라가 두 대면 재부팅 사이에 짝이 조용히 뒤바뀔 수 있고 그러면 신호수가 엉뚱한 로봇을 몰게 됨. /dev/v4l/by-id/ 경로 같은 안정적인 식별자를 쓰면 해결되지만, 지금 camera_node 는 정수를 받으므로 설정만 바꾸는 게 아니라 파라미터를 바꾸는 일임. -->
- **Each robot's camera means each robot's inference.** `torch` and `mediapipe` load per process, so N robots means N copies of the model running next to a mesh-heavy Gazebo. This has only ever been run with one, so the cost of two is not measured.
  <!-- 로봇마다 카메라가 있다는 건 로봇마다 추론이 돈다는 뜻임. torch 와 mediapipe 는 프로세스마다 올라가므로, 로봇 N대면 메시가 많은 Gazebo 옆에서 모델 N벌이 함께 돎. 지금까지 한 대로만 돌려 봤기 때문에 두 대일 때의 비용은 측정된 바 없음. -->

## History worth knowing

<!-- 알아 둘 만한 이력 -->

- **`Invalid mesh filename extension[.../world/navi_factory/__default__]` at startup is harmless and is not worth chasing again.** Seven of the world's eight `<actor>` elements carry no `<skin>`, and `__default__` is the SDF default for that filename, so gz resolves it against the world's own directory and fails — seven actors, seven error lines, printed once when the robot's lidar or camera first renders the scene. Measured in an isolated test world, the same skin-less actor still returns a lidar hit at its true distance: the skin load fails and gz falls through to the actor's `<link><visual>` mesh, which renders normally. The world alone prints nothing, and the robot alone in an empty world prints nothing; it takes both, because the server loads only collisions until something renders. `tools/tracks_to_actors.py` is what writes those actors. Left as is on purpose — silencing it means either giving the actors a skin (which may draw the person twice) or turning them into models (which loses their trajectories).
  <!-- 시작할 때 뜨는 Invalid mesh filename extension[.../world/navi_factory/__default__] 는 무해하고, 다시 추적할 가치가 없음. 월드의 actor 8개 중 7개에 skin 이 없고 __default__ 가 그 filename 의 SDF 기본값이라, gz 가 그걸 월드 폴더 기준으로 풀어 로드에 실패함 — 액터 7개에 에러 7줄이고, 로봇의 라이다·카메라가 씬을 처음 렌더할 때 한 번만 찍힘. 따로 만든 테스트 월드에서 재보니 그 skin 없는 액터도 실제 거리에서 라이다에 정상적으로 잡혔음: skin 로딩만 실패하고 gz 가 액터의 link visual 메시로 넘어가 그대로 렌더함. 월드만 띄우면 안 뜨고 빈 월드에 로봇만 띄워도 안 뜸 — 둘 다 있어야 뜨는데, 서버는 무언가 렌더하기 전까지 collision 만 읽기 때문. 이 액터들을 만드는 건 tools/tracks_to_actors.py 임. 일부러 그대로 둠 — 없애려면 액터에 skin 을 주거나(사람이 두 겹으로 그려질 수 있음) model 로 바꿔야 하는데(이동 경로를 잃음) 둘 다 손해가 더 큼. -->
- `signal_tracker_robot` — a fixed-base upper-body-only real-time motion-tracking robot — was scaffolded and then removed in commit `b65fdae`. It was a separate experiment, not the Phase 1 mobility robot, and is not part of the current tree.
  <!-- signal_tracker_robot — 고정 받침대형 상반신 전용 실시간 모션 트래킹 로봇 — 뼈대를 만들었다가 커밋 b65fdae 에서 제거됨. Phase 1 이동 로봇이 아닌 별도 실험이었고 현재 트리에는 없음. -->
- The vendored `aws-robomaker-small-warehouse-world-ros2` package was dropped from `src/`; its models now live under `worlds/navi_factory/models/`, so the old classic-Gazebo launch files it carried are no longer a porting concern.
  <!-- 벤더링했던 aws-robomaker-small-warehouse-world-ros2 패키지는 src/ 에서 제거됨. 그 모델들은 이제 worlds/navi_factory/models/ 아래에 있고, 그 패키지가 갖고 있던 구버전 Gazebo용 launch 파일들은 더 이상 포팅 대상이 아님. -->
- `gz sim` is started with `ExecuteProcess` rather than through `ros_gz_sim`'s `gz_sim.launch.py`. Through that wrapper the simulator quit silently a few seconds into this mesh-heavy world; called directly it keeps running.
  <!-- gz sim 은 ros_gz_sim 의 gz_sim.launch.py 를 거치지 않고 ExecuteProcess 로 직접 실행함. 그 래퍼를 통하면 메시가 많은 이 월드에서 몇 초 뒤 시뮬레이터가 조용히 종료되는 문제가 있었고, 직접 부르면 계속 정상 동작함. -->
- `camera_node` used to sit outside the robot loop as a single global node, on the assumption that one webcam would serve every robot, and it reached its robot through a hard-coded remap to `/robot1/cmd_vel_gesture`. After `robot1` was taken out of the launch that remap pointed at a namespace where no `twist_mux` was listening, so a recognized signal moved nothing. Making it one camera per robot inside the namespace removed the remap and with it the place where the two could drift apart.
  <!-- camera_node 는 웹캠 한 대가 모든 로봇을 담당한다는 전제로 로봇 루프 밖에 전역 노드 하나로 있었고, '/robot1/cmd_vel_gesture' 로 하드코딩된 리맵을 통해 자기 로봇에 닿았음. robot1 이 launch 에서 빠진 뒤로 그 리맵은 twist_mux 가 듣지 않는 네임스페이스를 가리켰고, 그래서 수신호를 인식해도 아무것도 움직이지 않았음. 로봇당 카메라 한 대로 바꿔 네임스페이스 안에 넣으면서 리맵이 사라졌고, 둘이 어긋날 수 있던 자리도 함께 없어짐. -->

Update this section whenever the robot description is replaced, the command interface is agreed with the vision repository, or Nav2 lands.
<!-- 로봇 모델이 교체되거나, 비전 리포지토리와 명령 인터페이스가 합의되거나, Nav2가 들어오면 이 섹션을 갱신할 것. -->
