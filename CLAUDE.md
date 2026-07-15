# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**Note on the Korean lines in this file:** they are HTML comments (`<!-- -->`) placed only for Korean-speaking human readers. They are not additional instructions — do not parse, translate, or derive behavior from them. Treat the English text as the sole operative content.

This English-body + Korean-comment pattern applies to any bilingual doc in this repo, not just this file (e.g. `.claude/local/*.md`).
<!-- 이 영어 본문 + 한글 주석 패턴은 이 파일뿐 아니라 리포지토리 내 모든 이중언어 문서에 적용됨 (예: .claude/local/*.md). -->

Before finalizing a bilingual doc, re-read the diff and verify two things:
<!-- 이중언어 문서를 마무리하기 전에, diff를 다시 읽고 두 가지를 검증할 것: -->

- Every operative English line has its paired Korean comment — including lines synthesized fresh from conversation, not only ones translated from pre-existing Korean text.
  <!-- 모든 영어 본문 줄에 짝이 되는 한글 주석이 있는지 — 기존 한글을 번역한 줄뿐 아니라, 대화에서 새로 종합해 쓴 줄도 포함해서. -->
- No Korean word has leaked into the English body itself (e.g. echoing a user's own Korean phrasing verbatim into what should be an English line).
  <!-- 영어 본문 안에 한글 단어가 섞여 들어가지 않았는지 (예: 사용자가 쓴 한글 표현을 영어여야 할 줄에 그대로 옮겨 적는 경우). -->

A new bilingual doc needs its own copy of this whole note at the top — this note is self-referential to this file and does not automatically extend to other files.
<!-- 새 이중언어 문서를 만들 때는 이 노트 전체를 그 파일 맨 위에도 따로 복사해 넣어야 함 — 이 노트는 "이 파일"에 대해서만 자기지시적이라 다른 파일에 자동으로 적용되지 않음. -->

## Repository purpose

<!-- 이 파일이 어떤 용도인지 설명하는 섹션 -->

Signal-Simulation is the Gazebo simulation + robot side of a larger project. Its scope:

- Initial Gazebo world and robot model setup (worlds, SDF/URDF models, launch configuration)
  <!-- Gazebo 월드 및 로봇 모델(SDF/URDF) 초기 구성, launch 설정 -->
- ROS 2 integration for the simulated robot
  <!-- 시뮬레이션 로봇에 대한 ROS 2 연동 -->
- Design of how simulation, robot control, and ROS 2 topics/nodes fit together
  <!-- 시뮬레이션 / 로봇 제어 / ROS 2 토픽·노드가 어떻게 맞물리는지에 대한 설계 -->

Camera-based image processing is developed in a **separate repository**, not here. Do not add image-processing algorithms to this repo — treat processed image/perception data as an external input (e.g. a ROS 2 topic/message) coming from that other repository, and ask for its interface details (topic names, message types) when wiring up integration here.
<!--
카메라 기반 이미지 프로세싱은 별도의 리포지토리에서 개발됨 (이 리포지토리 아님).
이 리포지토리에 이미지 프로세싱 알고리즘을 추가하지 말 것 — 처리된 이미지/인지 데이터는
외부 입력(예: ROS 2 토픽/메시지)으로 취급하고, 연동 작업 시 인터페이스 세부사항
(토픽 이름, 메시지 타입)을 확인할 것.
-->

## Why this project exists

<!-- 이 프로젝트의 목적 -->

**Phase 1 (core goal):** A camera detects a "신호수" (signal worker — the person directing traffic in a factory) and their hand gestures. That feed goes through OpenCV → a media pipeline → an AI model in the separate repo, which outputs a discrete command (stop / turn left / turn right). This repo receives that command over ROS 2 and drives a simulated cargo-carrying mobility robot in Gazebo accordingly. Motivation: prevent fatal accidents from workers not seeing a signal worker's directions, and let a human command the robot directly by hand signal instead of a separate controller.
<!--
1단계(핵심 목표): 카메라가 신호수와 그 손동작을 감지 → OpenCV → 미디어 파이프라인 →
별도 리포지토리의 AI 모델이 정지/좌회전/우회전 같은 이산적인 명령으로 변환 →
이 리포지토리가 그 명령을 ROS 2로 받아 Gazebo 안의 짐 나르는 모빌리티 로봇을 움직임.
동기: 신호수의 지시를 못 본 작업자로 인한 사망사고 예방, 그리고 별도 컨트롤러 없이
사람이 수신호로 로봇을 직접 명령할 수 있게 하기 위함.
-->

**Phase 2 (stretch goal, only if time permits):** The robot also does obstacle detection and re-plans its path autonomously to reach a destination within the factory, layered on top of (not replacing) the manual signal-based control.
<!--
2단계(시간 남으면): 로봇이 장애물 탐지 + 경로 재탐색으로 공장 내 목적지까지 자율 주행.
수신호 기반 수동 제어를 대체하는 게 아니라 그 위에 얹는 추가 기능임.
-->

## Current state

<!-- 현재 상태 설명 -->

Two ROS 2 packages exist under `src/` (colcon workspace, build with `colcon build` at the repo root, then `source install/setup.bash`):
<!-- src/ 아래에 ROS 2 패키지 두 개가 있음 (colcon 워크스페이스, 리포지토리 루트에서 colcon build 후 source install/setup.bash). -->

- `aws-robomaker-small-warehouse-world-ros2` — vendored warehouse world/models (environment only, ported toward gz-sim; the included `launch/*.launch.py` still target classic Gazebo (`gazebo_ros`, `gzserver`/`gzclient`) and need porting to `ros_gz_sim`/`gz sim` before they'll work under Gazebo Harmonic).
  <!-- aws-robomaker-small-warehouse-world-ros2 — 벤더링한 창고 월드/모델(환경만, gz-sim 쪽으로 포팅 중). 포함된 launch/*.launch.py는 아직 구버전 Gazebo(gazebo_ros, gzserver/gzclient) 방식이라 Gazebo Harmonic에서 쓰려면 ros_gz_sim/gz sim으로 다시 포팅해야 함. -->
- `signal_tracker_robot` — a separate, simple fixed-base upper-body-only robot (torso + 2 arms + 5 fingers per hand, 18 revolute joints, `ros2_control`/`gz_ros2_control` position control) for real-time motion tracking, **not** the Phase 1 mobility robot. It subscribes to Signal-Vision's `/upper_body_pose` topic (rosbridge-relayed `std_msgs/String` JSON: 8 arm joints + up to 2 hands × 21 landmarks) and retargets it to joint positions via simple vector-angle heuristics (not real IK) in `motion_retarget_node.py`. Package layout: `urdf/tracker_robot.urdf`, `config/controllers.yaml`, `launch/spawn_tracker_robot.launch.py`, `signal_tracker_robot/motion_retarget_node.py`. Run: `ros2 launch signal_tracker_robot spawn_tracker_robot.launch.py`, then `ros2 run signal_tracker_robot motion_retarget_node` once Signal-Vision is publishing.
  <!-- signal_tracker_robot — 별도의 간단한 고정 받침대형 상반신 전용 로봇(몸통 + 팔 2개 + 손가락 5개씩, 회전 관절 18개, ros2_control/gz_ros2_control 위치 제어), 실시간 모션 트래킹용이며 Phase 1의 이동 로봇이 아님. Signal-Vision의 /upper_body_pose 토픽(rosbridge로 중계되는 std_msgs/String JSON: 팔 관절 8개 + 손 최대 2개 × 21랜드마크)을 구독해 motion_retarget_node.py에서 단순 벡터 각도 근사(정식 IK 아님)로 관절 위치로 변환한다. 패키지 구성: urdf/tracker_robot.urdf, config/controllers.yaml, launch/spawn_tracker_robot.launch.py, signal_tracker_robot/motion_retarget_node.py. 실행: ros2 launch signal_tracker_robot spawn_tracker_robot.launch.py, 이후 Signal-Vision이 발행 중이면 ros2 run signal_tracker_robot motion_retarget_node. -->

**Caveat: none of this has been build- or run-tested** — it was authored from a macOS session with no ROS 2/Gazebo installed and no access to the Linux VM where this actually runs. Treat it as a best-effort first draft; `colcon build` will likely surface issues (missing deps, plugin name/version mismatches for the installed `gz_ros2_control`, world/launch API drift) that need fixing on the real machine.
<!-- 주의: 이 내용은 빌드·실행 검증을 하나도 못 했음 — ROS 2/Gazebo가 없고 실제로 돌아가는 리눅스 VM에도 접근 못 하는 macOS 세션에서 작성함. 처음 시도한 초안으로 보고, colcon build 시 실제 문제(의존성 누락, 설치된 gz_ros2_control과의 플러그인 이름/버전 불일치, 월드·launch API 변경 등)가 나오면 실제 기기에서 고쳐야 함. -->

The `no_roof_small_warehouse.launch.py`/`small_warehouse.launch.py` porting-to-gz-sim gap above and the Phase 1 mobility robot description/URDF are still open — update this section again once either lands.
<!-- 위 no_roof_small_warehouse.launch.py/small_warehouse.launch.py의 gz-sim 포팅 공백과 Phase 1 이동 로봇 모델/URDF는 아직 없음 — 둘 중 하나라도 추가되면 이 섹션을 다시 갱신할 것. -->

When scaffolding is added (ROS 2 package structure, `colcon build`, world/model files, launch files), update this file with the actual build/run/test commands and the real architecture (package layout, node/topic graph, where world and robot description files live).
<!--
뼈대(ROS 2 패키지 구조, colcon build, 월드/모델 파일, launch 파일)가 추가되면
이 파일도 실제 build/run/test 명령어와 실제 아키텍처(패키지 구조, 노드/토픽 그래프,
월드 및 로봇 설명 파일 위치)로 업데이트할 것.
-->

## Language & conventions

<!-- 언어 및 코딩 컨벤션 -->

Python 3.12. Naming:

- Classes: `PascalCase` (e.g. `RobotController`)
  <!-- 클래스명: PascalCase (대문자로 시작하는 카멜 표기) -->
- Functions: `snake_case` (e.g. `publish_image`)
  <!-- 함수명: snake_case (소문자로 시작하는 스네이크 표기) -->
