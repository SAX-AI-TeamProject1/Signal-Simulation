# AGENT.md — Project structure

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. It covers the **concrete code structure**: workspace layout, packages, files, and build/run commands. For project purpose and current status see `doc/design.md`; for the ML training process see `doc/ml-flow.md`.

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

This is a colcon (ROS 2) workspace. Packages live under `src/`; `build/`, `install/`, `log/` are build artifacts and are gitignored.
<!-- colcon(ROS 2) 워크스페이스임. 패키지는 src/ 아래에 있고, build/ · install/ · log/ 는 빌드 산출물이라 gitignore됨. -->

```
Signal-Simulation/
├── CLAUDE.md              # personal local prompt (gitignored); @imports doc/*.md
├── doc/                   # team-shared prompts (committed)
│   ├── AGENT.md           # this file — project structure
│   ├── design.md          # project overview & current status
│   └── ml-flow.md         # machine-learning training process
└── src/
    └── aws-robomaker-small-warehouse-world-ros2/   # vendored warehouse environment
```
<!-- 위 트리: 루트에 개인 로컬 CLAUDE.md, 팀 공용 doc/, 그리고 src/ 아래 워크스페이스 패키지. -->

## Packages

<!-- 패키지 -->

### `aws-robomaker-small-warehouse-world-ros2`

Vendored warehouse world/models — environment only, no robot. Key contents:
<!-- 벤더링한 창고 월드/모델 — 환경만 있고 로봇은 없음. 주요 구성: -->

- `worlds/` — world files (`no_roof_small_warehouse`, `small_warehouse`)
  <!-- worlds/ — 월드 파일 (no_roof_small_warehouse, small_warehouse) -->
- `models/` — warehouse props (shelves, pallets, buckets, walls, lamps, etc.), each an SDF model directory
  <!-- models/ — 창고 소품(선반, 팔레트, 버킷, 벽, 램프 등), 각각 SDF 모델 디렉터리 -->
- `launch/` — `no_roof_small_warehouse.launch.py`, `small_warehouse.launch.py` (still classic-Gazebo based; see `doc/design.md` for the gz-sim porting gap)
  <!-- launch/ — 위 두 launch 파일 (아직 구버전 Gazebo 기반; gz-sim 포팅 공백은 doc/design.md 참고) -->
- `maps/`, `rviz/` — navigation maps and an RViz config
  <!-- maps/, rviz/ — 내비게이션 맵과 RViz 설정 -->
- `port_aws_warehouse_to_gz.py`, `dedup_world_lights_plugins.py` — helper scripts for the gz-sim port
  <!-- port_aws_warehouse_to_gz.py, dedup_world_lights_plugins.py — gz-sim 포팅용 보조 스크립트 -->

No other packages exist yet. The Phase 1 mobility robot package/URDF has not been created; the previously scaffolded `signal_tracker_robot` package was removed (see `doc/design.md`).
<!-- 아직 다른 패키지는 없음. Phase 1 이동 로봇 패키지/URDF는 미생성, 이전의 signal_tracker_robot 패키지는 제거됨 (doc/design.md 참고). -->

## Build & run

<!-- 빌드 및 실행 -->

From the repo root:
<!-- 리포지토리 루트에서: -->

```bash
colcon build
source install/setup.bash
```
<!-- colcon build 후 install/setup.bash 를 source 함. -->

When new scaffolding is added (packages, world/model files, launch files), update this file with the actual build/run/test commands and the real node/topic graph.
<!-- 새 뼈대(패키지, 월드/모델 파일, launch 파일)가 추가되면 실제 build/run/test 명령어와 실제 노드/토픽 그래프로 이 파일을 갱신할 것. -->

## Language & conventions

<!-- 언어 및 코딩 컨벤션 -->

Python 3.12. Naming:
<!-- Python 3.12. 명명 규칙: -->

- Classes: `PascalCase` (e.g. `RobotController`)
  <!-- 클래스명: PascalCase (대문자로 시작하는 카멜 표기) -->
- Functions: `snake_case` (e.g. `publish_image`)
  <!-- 함수명: snake_case (소문자로 시작하는 스네이크 표기) -->
