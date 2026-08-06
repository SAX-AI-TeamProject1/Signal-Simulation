#!/usr/bin/env bash
# Opens rqt_image_view on the object-detection image published by detect_node.
# Used by the VS Code task "10. 판별 영상 보기 (rqt_image_view)".
#
# detect_node 는 <ns>/detect_image 에 구독자가 없으면 추론 자체를 건너뛴다
# (주행 성능을 건드리지 않으려는 설계). 그래서 이 뷰어를 띄우는 것이 곧
# "판별을 켜는" 동작이고, 창을 닫으면 추론도 함께 멈춘다.
#
# 토픽을 인자로 바꿀 수 있다:
#   ./scripts/run_detect_view.sh /robot2/detect_image
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

TOPIC="${1:-/robot2/detect_image}"

if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
    echo "[run_detect_view] install/setup.bash 가 없습니다." >&2
    echo "[run_detect_view] 먼저 태스크 '1. 워크스페이스 빌드' 를 실행하세요." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"

# rqt_image_view 는 ros-jazzy-desktop 에 들어 있지만, ros-base 만 깔린 머신도
# 있을 수 있어 없으면 원인을 분명히 알려 준다.
if ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
    echo "[run_detect_view] rqt_image_view 패키지가 없습니다." >&2
    echo "[run_detect_view] sudo apt-get install -y ros-jazzy-rqt-image-view" >&2
    exit 1
fi

# 발행자가 없으면 창은 뜨지만 영원히 회색이라, 원인을 먼저 짚어 준다.
# (detect_node 는 enable_detect:=true 로 켜야 뜬다 — 기본은 꺼짐)
if ! ros2 topic info "$TOPIC" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "[run_detect_view] '$TOPIC' 에 발행자가 없습니다." >&2
    echo "[run_detect_view] 태스크 '9. bringup + 물체 판별' 로 먼저 띄우세요" >&2
    echo "[run_detect_view] (또는 bringup 에 enable_detect:=true 를 주세요)." >&2
    echo "[run_detect_view] 그래도 창은 띄웁니다 — 노드가 늦게 뜨면 자동으로 그려집니다." >&2
fi

echo "[run_detect_view] $TOPIC 를 엽니다."
exec ros2 run rqt_image_view rqt_image_view "$TOPIC"
