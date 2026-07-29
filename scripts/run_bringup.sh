#!/usr/bin/env bash
# Runs robot_control/launch/bringup.launch.py — rsp + gz sim + robot spawn +
# flat_ground + ros_gz_bridge + twist_mux (+ camera_node when enable_camera:=true).
# Used by VS Code task "7. bringup 실행 (로봇 + Gazebo + 브릿지)".
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    echo "[run_bringup] install/setup.bash 가 없습니다." >&2
    echo "[run_bringup] 먼저 태스크 '1. 워크스페이스 빌드' 를 실행하세요." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"

# twist_mux 는 자작 노드가 아니라 apt 설치 패키지다(ros-jazzy-twist-mux). build_workspace.sh는
# ros2가 이미 PATH에 있는(=이미 한 번 셋업된) 머신에서는 apt install 블록 전체를 건너뛰므로,
# 이 패키지가 나중에 목록에 추가돼도 기존 머신엔 소급 적용이 안 된다 — 그래서 bringup이
# 직접 쓰는 시점에 없으면 여기서 바로 설치한다. (sudo 비밀번호가 필요할 수 있다)
if ! ros2 pkg prefix twist_mux >/dev/null 2>&1; then
    echo "[run_bringup] twist_mux 패키지가 없습니다 — 설치합니다 (sudo 비밀번호 필요할 수 있음)." >&2
    sudo apt-get install -y ros-jazzy-twist-mux
    source /opt/ros/jazzy/setup.bash   # ament 인덱스 갱신 반영
fi

# bringup 의 camera_node 는 launch_ros 가 시스템 python3 로 띄운다. camera_node/inference.py 는
# robot_control/vision_hand(벤더 복사본)를 import하므로 PYTHONPATH 조작은 필요 없다 — 다만
# torch/mediapipe 같은 실제 라이브러리는 시스템 python3.12에 설치돼 있어야 한다
# (scripts/setup_infer_env.py, 태스크 4가 자동으로 설치한다).
if ! python3 -c "import torch, mediapipe" >/dev/null 2>&1; then
    echo "[run_bringup] 시스템 python3.12에 torch/mediapipe가 없습니다 → camera_node는 실패할 수 있습니다." >&2
    echo "[run_bringup] 태스크 '4. Vision 패키지 설치'를 먼저 실행하거나, 시뮬만 볼 거면 enable_camera:=false로 실행하세요." >&2
fi

# 이전 실행의 gz 서버가 남아 있으면 월드가 두 번 뜨므로 정리 (run_gazebo.sh 와 동일)
pkill -TERM -f "gz sim server" 2>/dev/null || true
sleep 1

# 인자는 그대로 launch 로 넘긴다. 예:
#   ./scripts/run_bringup.sh enable_camera:=false headless:=true
exec ros2 launch robot_control bringup.launch.py "$@"
