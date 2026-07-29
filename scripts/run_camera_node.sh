#!/usr/bin/env bash
# Runs robot_control's camera_node alone — no Gazebo, no robot, no bridge.
# Unit check for the webcam -> gesture inference -> /gesture + cmd_vel_gesture path.
# Used by VS Code task "6. camera node 단독 실행 (웹캠 + 제스처 추론만)".
#
# Why a script instead of a single task command: this needs 3 steps in one shell —
# ROS 2 env, workspace overlay, then the .venv-infer interpreter.
#
# Why .venv-infer and NOT .venv-perception:
#   camera_node/inference.py imports src.capture.extractor and src.inference.predict.
#   Those live in Signal-Vision (.venv-infer, task 4). Signal-transport-perception
#   (.venv-perception, task 5) only ships src/infer.py (YOLO) — different API.
#   .venv-infer also pins numpy 1.26.4, matching the numpy that ROS Jazzy's
#   cv_bridge boost extension was compiled against; .venv-perception's numpy 2.3.5
#   makes `from cv_bridge import CvBridge` fail outright.
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

VENV_PY="$REPO_ROOT/.venv-infer/bin/python"
if [ ! -x "$VENV_PY" ]; then
    echo "[run_camera_node] .venv-infer 가 없습니다." >&2
    echo "[run_camera_node] 먼저 태스크 '4. Vision 패키지 설치' 를 실행하세요." >&2
    exit 1
fi

if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    echo "[run_camera_node] install/setup.bash 가 없습니다." >&2
    echo "[run_camera_node] 먼저 태스크 '1. 워크스페이스 빌드' 를 실행하세요." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"

# camera_node 의 device_id 기본값은 0 이지만 /dev/video0 이 늘 있는 건 아니다
# (이 머신은 video1/video2 만 존재). 사용자가 device_id 를 직접 주지 않았을 때만
# 존재하는 가장 낮은 번호의 /dev/videoN 을 찾아 넣는다.
EXTRA_ARGS=()
if [[ "$*" != *device_id* ]]; then
    for dev in /dev/video*; do
        [ -e "$dev" ] || continue          # 매칭 0건이면 리터럴 '/dev/video*' 가 들어온다
        DEV_ID="${dev#/dev/video}"
        echo "[run_camera_node] 웹캠 자동 선택: $dev (device_id:=$DEV_ID)" >&2
        echo "[run_camera_node] 다른 장치를 쓰려면: $0 --ros-args -p device_id:=<N>" >&2
        EXTRA_ARGS=(--ros-args -p "device_id:=$DEV_ID")
        break
    done
fi

# 인자는 그대로 넘긴다. 예: 토픽 리맵을 걸어 bringup 과 같은 배선으로 확인
#   ./scripts/run_camera_node.sh --ros-args -r cmd_vel_gesture:=/robot1/cmd_vel_gesture
exec "$VENV_PY" -m robot_control.camera_node.node "${EXTRA_ARGS[@]}" "$@"
