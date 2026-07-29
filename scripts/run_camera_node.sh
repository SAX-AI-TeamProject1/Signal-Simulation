#!/usr/bin/env bash
# Runs robot_control's camera_node alone — no Gazebo, no robot, no bridge.
# Unit check for the webcam -> gesture inference -> /gesture + cmd_vel_gesture path.
# Used by VS Code task "6. camera node 단독 실행 (웹캠 + 제스처 추론만)".
#
# Why system python3.12 and NOT .venv-infer/.venv-perception:
#   camera_node/inference.py imports robot_control.vision_hand.capture.extractor and
#   robot_control.vision_hand.inference.predict — a vendored copy of Signal-Vision's
#   src/ living inside this package (see scripts/setup_infer_env.py), not the pip
#   package itself. So the only runtime requirement is that system python3.12 has
#   torch/mediapipe/numpy/opencv-contrib-python installed directly (see that script's
#   header comment for the install command) — no venv needed, and none would help:
#   colcon always builds/runs ament_python console scripts with system python3.12
#   regardless of which venv is active, so `ros2 run` / this script could never reach
#   a venv's site-packages anyway.
#   .venv-perception is a dead end here on top of that: Signal-transport-perception
#   needs opencv-python, mediapipe needs opencv-contrib-python — different pip
#   packages that both install to `cv2/`, so they can't share one environment.
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    echo "[run_camera_node] install/setup.bash 가 없습니다." >&2
    echo "[run_camera_node] 먼저 태스크 '1. 워크스페이스 빌드' 를 실행하세요." >&2
    exit 1
fi

if ! python3 -c "import torch, mediapipe" >/dev/null 2>&1; then
    echo "[run_camera_node] 시스템 python3.12에 torch/mediapipe가 없습니다." >&2
    echo "[run_camera_node] 태스크 '4. Vision 패키지 설치'를 실행해 robot_control/vision_hand를 최신화한 뒤," >&2
    echo "[run_camera_node] torch/mediapipe/numpy==1.26.4/opencv-contrib-python을 시스템 python3.12에 pip 설치하세요" >&2
    echo "[run_camera_node] (scripts/setup_infer_env.py 헤더 주석 참고)." >&2
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
exec ros2 run robot_control camera_node "${EXTRA_ARGS[@]}" "$@"
