# Last updated: 2026-07-28
"""Signal-Vision을 pip으로 설치한 venv를 만든다.

--system-site-packages로 만들어서, ROS 2가 apt로 깐 rclpy/cv_bridge 같은 시스템 site-packages를
그대로 상속한다 — 그래야 robot_control의 camera_node.py(rclpy 필요)가 이 venv의 파이썬으로 실행되면서
동시에 여기서 pip 설치한 mediapipe/torch/signal-vision도 import할 수 있다. ROS가 없는 머신
(예: 이 저장소를 막 clone한 macOS/Windows)에서는 시스템에 상속할 rclpy가 애초에 없으므로 이 플래그는
그냥 아무 영향이 없다 — 순수 설치 확인용으로 그대로 써도 무방하다.
이 스크립트 자체를 실행하는 인터프리터 버전은 상관없다 (python3.12는 내부에서 직접 찾는다).

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 v*.*.* 태그를
`git ls-remote`로 조회해서 그걸 설치한다 — 예전처럼 스크립트에 박아둔 버전이 실제
최신 fix보다 뒤처지는 걸 막기 위함 (예: mediapipe/numpy/opencv aarch64 호환성 수정).
"""
import sys
from pathlib import Path
from setup_repo_venv import install

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-Vision.git"
VENV_DIR = Path(__file__).resolve().parent.parent / ".venv-infer"

if __name__ == "__main__":
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    raise SystemExit(install(
        REPO_URL, VENV_DIR,
        verify_hint="from src.capture.extractor import FeatureExtractor; print('OK')",
        ref=ref, system_site_packages=True,
    ))