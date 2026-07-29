# Last updated: 2026-07-29
"""Signal-Vision을 pip으로 설치한 venv를 만들고, 그 소스를 robot_control로 vendor 복사한다.

--system-site-packages로 만들어서, ROS 2가 apt로 깐 rclpy/cv_bridge 같은 시스템 site-packages를
그대로 상속한다 — 그래야 robot_control의 camera_node.py(rclpy 필요)가 이 venv의 파이썬으로 실행되면서
동시에 여기서 pip 설치한 mediapipe/torch/signal-vision도 import할 수 있다. ROS가 없는 머신
(예: 이 저장소를 막 clone한 macOS/Windows)에서는 시스템에 상속할 rclpy가 애초에 없으므로 이 플래그는
그냥 아무 영향이 없다 — 순수 설치 확인용으로 그대로 써도 무방하다.
이 스크립트 자체를 실행하는 인터프리터 버전은 상관없다 (python3.12는 내부에서 직접 찾는다).

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 v*.*.* 태그를
`git ls-remote`로 조회해서 그걸 설치한다 — 예전처럼 스크립트에 박아둔 버전이 실제
최신 fix보다 뒤처지는 걸 막기 위함 (예: mediapipe/numpy/opencv aarch64 호환성 수정).

설치가 끝나면 venv에 pip으로 깔린 Signal-Vision의 `src/` 패키지를
robot_control/robot_control/vision_hand/ 로 그대로 복사한다(vendor). camera_node.py는
`from src...`가 아니라 `from robot_control.vision_hand...`로 import한다 — Signal-Vision의
최상위 패키지명이 하필 `src`라서, 이 레포 자체의 src/ 디렉터리와 이름이 겹치는 걸 피하기
위함이다. 이 스크립트를 다시 돌릴 때마다 vision_hand/도 최신 태그 기준으로 통째로 교체되므로
수동으로 다시 복사할 필요가 없다.

복사한 파일들 내부에도 서로를 `from src.xxx import ...`로 참조하는 절대 import가 섞여
있어서(원래 패키지명이 src였으니 당연함), 그대로 두면 vision_hand 안의 파일들이 서로가
아니라 시스템/venv에 pip으로 깔린 별개의 src 패키지를 참조하게 된다 — 두 사본이 버전
드리프트로 갈라져도 아무도 못 눈치채는 상황이 생긴다. 그래서 복사 직후 vision_hand/ 안의
모든 `from src.` / `import src.` 를 `from robot_control.vision_hand.` /
`import robot_control.vision_hand.` 로 일괄 치환해서, vision_hand가 pip src 패키지 없이도
완전히 자기 완결적으로 동작하게 만든다.
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

from setup_repo_venv import install, venv_python

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-Vision.git"
REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv-infer"
VENDOR_DEST = REPO_ROOT / "src" / "robot_control" / "robot_control" / "vision_hand"

# 줄 시작(들여쓰기 허용)의 "from src." / "import src." / "import src " 형태만 건드린다.
# 문자열/주석 안의 우연한 "src" 언급까지 건드리지 않도록 import 문 자리로 한정.
_SRC_IMPORT_RE = re.compile(r'^(\s*(?:from|import)\s+)src(\.|(?=\s))', re.MULTILINE)


def _rewrite_internal_imports() -> None:
    for py_file in VENDOR_DEST.rglob("*.py"):
        text = py_file.read_text()
        patched = _SRC_IMPORT_RE.sub(r'\1robot_control.vision_hand\2', text)
        if patched != text:
            py_file.write_text(patched)


def _vendor_copy() -> None:
    py = str(venv_python(VENV_DIR))
    result = subprocess.run(
        [py, "-c", "import src, pathlib; print(pathlib.Path(src.__file__).parent)"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise SystemExit(f"vendor 복사 실패: venv에서 src 패키지를 찾을 수 없습니다\n{result.stderr}")

    src_pkg = Path(result.stdout.strip())
    if VENDOR_DEST.exists():
        shutil.rmtree(VENDOR_DEST)
    shutil.copytree(src_pkg, VENDOR_DEST, ignore=shutil.ignore_patterns("__pycache__"))
    _rewrite_internal_imports()
    print(f"vendor 복사 완료: {src_pkg} -> {VENDOR_DEST} (내부 src.* import도 robot_control.vision_hand.*로 치환)")


if __name__ == "__main__":
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    rc = install(
        REPO_URL, VENV_DIR,
        verify_hint="from src.capture.extractor import FeatureExtractor; print('OK')",
        ref=ref, system_site_packages=True,
    )
    if rc == 0:
        _vendor_copy()
    raise SystemExit(rc)
