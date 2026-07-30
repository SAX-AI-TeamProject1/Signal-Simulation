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

# 이전 bringup 실행이 아직 돌고 있으면 (예: 태스크를 정지 안 하고 다시 실행) gz 서버가
# 두 번 뜨거나, 이전 camera_node 가 웹캠을 물고 있어서 새 인스턴스가 못 여는 등 꼬인다.
# VS Code 태스크 패널을 재사용하는 것만으로는 launch 가 띄운 자식 프로세스(bridge_node,
# twist_mux, camera_node 등)까지 다 안 죽는 경우가 있어 여기서 명시적으로 정리한다.
# ros2 launch 자체에 SIGINT 를 보내면(강제 kill 이 아니라) launch 가 자기 자식들에게
# 알아서 순서대로 SIGINT 를 돌려 정상 종료시킨다 — bridge_node 가 강제 종료보다
# 정상 종료일 때 훨씬 안정적으로 죽는다(그냥 kill -9 하면 segfault 로 죽는 걸 본 적 있음).
OLD_PID=$(pgrep -f "ros2 launch robot_control bringup.launch.py" | head -1)
if [ -n "$OLD_PID" ]; then
    echo "[run_bringup] 이전 bringup(pid $OLD_PID)이 아직 돌고 있어 정상 종료시킵니다..." >&2
    kill -INT "$OLD_PID" 2>/dev/null || true
    for _ in $(seq 1 20); do   # 최대 10초 대기
        kill -0 "$OLD_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -0 "$OLD_PID" 2>/dev/null && kill -KILL "$OLD_PID" 2>/dev/null || true
fi
# gz 서버는 launch 의 자식이 아니라 IncludeLaunchDescription 이 띄운 별도 그룹이라
# 위 SIGINT 캐스케이드로도 안 죽는 경우가 있어 별도로 한 번 더 정리 (run_gazebo.sh 와 동일).
pkill -TERM -f "gz sim server" 2>/dev/null || true
sleep 1

# 인자는 그대로 launch 로 넘긴다. 예:
#   ./scripts/run_bringup.sh enable_camera:=false headless:=true
exec ros2 launch robot_control bringup.launch.py "$@"
