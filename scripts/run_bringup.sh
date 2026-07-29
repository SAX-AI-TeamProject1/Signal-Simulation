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

# bringup 의 camera_node 는 launch_ros 가 시스템 python3 로 띄우므로, 그 상태로는
# src.capture / src.inference (Signal-Vision) 를 import 하지 못한다.
# .venv-infer 의 site-packages 를 PYTHONPATH 앞에 붙여 import 가능하게 만든다.
# .venv-perception 이 아니라 .venv-infer 인 이유는 run_camera_node.sh 주석 참고.
# venv 가 없으면 그냥 건너뛴다 — enable_camera:=false 로 시뮬만 돌릴 수 있어야 하므로.
VENV_SP=$(echo "$REPO_ROOT"/.venv-infer/lib/python3.*/site-packages)
if [ -d "$VENV_SP" ]; then
    export PYTHONPATH="$VENV_SP${PYTHONPATH:+:$PYTHONPATH}"
else
    echo "[run_bringup] .venv-infer 없음 → camera_node 는 실패할 수 있습니다." >&2
    echo "[run_bringup] 시뮬만 볼 거면 enable_camera:=false 로 실행하세요." >&2
fi

# 이전 실행의 gz 서버가 남아 있으면 월드가 두 번 뜨므로 정리 (run_gazebo.sh 와 동일)
pkill -TERM -f "gz sim server" 2>/dev/null || true
sleep 1

# 인자는 그대로 launch 로 넘긴다. 예:
#   ./scripts/run_bringup.sh enable_camera:=false headless:=true
exec ros2 launch robot_control bringup.launch.py "$@"
