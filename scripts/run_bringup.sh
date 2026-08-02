#!/usr/bin/env bash
# Runs knavi_bringup/launch/bringup.launch.py — rsp + gz sim + robot spawn +
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
# signal_vision/vision_hand(벤더 복사본)를 import하므로 PYTHONPATH 조작은 필요 없다 — 다만
# torch/mediapipe 같은 실제 라이브러리는 시스템 python3.12에 설치돼 있어야 한다
# (scripts/setup_infer_env.py, 태스크 4가 자동으로 설치한다).
if ! python3 -c "import torch, mediapipe" >/dev/null 2>&1; then
    echo "[run_bringup] 시스템 python3.12에 torch/mediapipe가 없습니다 → camera_node는 실패할 수 있습니다." >&2
    echo "[run_bringup] 태스크 '4. Vision 패키지 설치'를 먼저 실행하거나, 시뮬만 볼 거면 enable_camera:=false로 실행하세요." >&2
fi

# marker_vision(TrackMarker 감속 노드)도 camera_node와 같은 이유로 시스템 python3에
# ultralytics가 있어야 한다. enable_marker_vision 기본값이 false라 대부분은 해당 없음.
if ! python3 -c "import ultralytics" >/dev/null 2>&1; then
    echo "[run_bringup] 시스템 python3.12에 ultralytics가 없습니다 → enable_marker_vision:=true 로 켜면 marker_vision이 실패합니다." >&2
    echo "[run_bringup] pip install ultralytics 로 설치하거나, 기본값(false)대로 꺼두세요." >&2
fi

# 이전 bringup 실행이 아직 돌고 있으면 (예: 태스크를 정지 안 하고 다시 실행) gz 서버가
# 두 번 뜨거나, 이전 camera_node 가 웹캠을 물고 있어서 새 인스턴스가 못 여는 등 꼬인다.
# VS Code 태스크 패널을 재사용하는 것만으로는 launch 가 띄운 자식 프로세스(bridge_node,
# twist_mux, camera_node 등)까지 다 안 죽는 경우가 있어 여기서 명시적으로 정리한다.
# ros2 launch 자체에 SIGINT 를 보내면(강제 kill 이 아니라) launch 가 자기 자식들에게
# 알아서 순서대로 SIGINT 를 돌려 정상 종료시킨다 — bridge_node 가 강제 종료보다
# 정상 종료일 때 훨씬 안정적으로 죽는다(그냥 kill -9 하면 segfault 로 죽는 걸 본 적 있음).
#
# 부모(ros2 launch) 하나만 찾아 죽이는 것만으로는 부족하다는 게 실측으로 드러났다
# (2026-07-30): launch 부모가 먼저 죽어버리는 경우(태스크 패널을 정지 버튼이 아닌
# 다른 방식으로 끊는 등) twist_mux/bridge_node/waypoint_follower/robot_state_publisher
# 자식들이 orphan 으로 남는데, 그러면 "ros2 launch ..." 패턴이 더 이상 안 잡혀 아래
# 정리 블록 전체가 스킵되고 좀비가 무한정(관측상 8세대까지) 쌓인다. 그래서 부모
# 생사와 무관하게 이 launch 가 띄우는 노드들을 이름으로 직접, 매번 정리한다.
OLD_PIDS=$(pgrep -f "ros2 launch knavi_bringup bringup.launch.py" || true)
if [ -n "$OLD_PIDS" ]; then
    echo "[run_bringup] 이전 bringup(pid: $(echo $OLD_PIDS | tr '\n' ' '))이 아직 돌고 있어 정상 종료시킵니다..." >&2
    kill -INT $OLD_PIDS 2>/dev/null || true
    for _ in $(seq 1 20); do   # 최대 10초 대기
        pgrep -f "ros2 launch knavi_bringup bringup.launch.py" >/dev/null 2>&1 || break
        sleep 0.5
    done
    pkill -KILL -f "ros2 launch knavi_bringup bringup.launch.py" 2>/dev/null || true
fi

# gz 서버/GUI 와, 부모 없이 orphan 으로 남을 수 있는 이 launch 전용 노드들
# (bridge_node, twist_mux, waypoint_follower, robot_state_publisher, camera_node)을
# 이름으로 한 번 더 확실히 정리한다 — 위 부모 kill 로 이미 죽었으면 아무 것도 안 걸린다.
# 먼저 정상 종료(TERM)를 시도해 bridge_node 강제종료 크래시를 피하고, 잠깐 기다린 뒤
# 그래도 살아있으면 KILL 로 마무리한다.
#
# camera_node 를 넣는 이유: 위 38행 주석이 정리의 동기로 든 증상("이전 camera_node 가
# 웹캠을 물고 있어서 새 인스턴스가 못 연다")의 당사자인데 정작 목록에 빠져 있었다.
# 부작용은 감수한다 — 태스크 6(run_camera_node.sh)으로 단독 실행 중인 camera_node 도
# 같이 죽는다. 어차피 웹캠은 한 대뿐이라 둘이 동시에 살아 있어도 뒤에 뜬 쪽이 실패한다.
# 패턴이 실행파일 경로 형태인 이유: launch_ros 가 띄우는 실체는
# install/signal_vision/lib/signal_vision/camera_node 라서 "camera_node" 만 쓰면
# 이 스크립트 자신이나 편집기 프로세스까지 걸릴 수 있다.
ORPHAN_PATTERNS=(
    "gz sim"
    "bridge_node"
    "twist_mux"
    "auto_drive/lib/auto_drive/waypoint_follower"
    "auto_drive/lib/auto_drive/mission_follower"
    "signal_vision/lib/signal_vision/camera_node"
    "robot_state_publisher.*robot_description"
)
for pattern in "${ORPHAN_PATTERNS[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 2
for pattern in "${ORPHAN_PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
done
sleep 1

# 인자는 그대로 launch 로 넘긴다. 예:
#   ./scripts/run_bringup.sh enable_camera:=false headless:=true
exec ros2 launch knavi_bringup bringup.launch.py "$@"
