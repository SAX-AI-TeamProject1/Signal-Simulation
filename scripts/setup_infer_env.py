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

마지막으로 torch/mediapipe/numpy/opencv-contrib-python을 **시스템 python3.12**에도
설치한다. .venv-infer 는 위 벤더 복사의 "출처"일 뿐 실행 환경이 아니다 — colcon이
빌드하는 console_scripts(ros2 run이 실행하는 것)는 venv가 활성화되어 있어도 항상
시스템 python3.12로 만들어진다(colcon 자체가 시스템 파이썬으로 설치돼 있어서). 그래서
vision_hand가 import 문 레벨에서는 자기 완결적이어도, 그 안에서 쓰는 torch/mediapipe
같은 실제 라이브러리는 시스템 python3.12에도 있어야 `ros2 run robot_control camera_node`
가 동작한다. 이미 설치돼 있으면 건너뛰므로 여러 머신에서 반복 실행해도 안전하다.
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

from setup_repo_venv import find_latest_tag, install, venv_python
from task_output import banner, step

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


def _install_system_runtime_deps(ref: str) -> int:
    """
    torch/mediapipe/numpy/opencv-contrib-python/Signal-Vision을 시스템 python3.12에 설치한다.

    이미 임포트가 되면(=이미 설치돼 있으면) 건너뛴다 — 매 머신마다, 매번 이 스크립트를
    다시 돌려도 무거운 pip install을 반복하지 않기 위해서다.
    """
    check = subprocess.run([sys.executable, "-c", "import torch, mediapipe"],
                           capture_output=True)
    if check.returncode == 0:
        step("시스템 python3.12에 torch/mediapipe 이미 설치됨 — 건너뜀")
        return 0

    step("시스템 python3.12에 torch/mediapipe 없음 — 설치 시작 (sudo 비밀번호 필요할 수 있음)")

    if subprocess.run([sys.executable, "-m", "pip", "--version"],
                      capture_output=True).returncode != 0:
        step("시스템 pip 없음 — apt install python3-pip")
        if subprocess.run(["sudo", "apt", "install", "-y", "python3-pip"]).returncode != 0:
            banner(False, "python3-pip 설치 실패")
            return 1

    step("torch/numpy/opencv-contrib-python/mediapipe/Signal-Vision 설치 중 (수 분 소요)")
    install_cmd = [
        "sudo", sys.executable, "-m", "pip", "install",
        "--break-system-packages", "--ignore-installed",
        "torch", "numpy==1.26.4", "opencv-contrib-python", "mediapipe",
        f"git+{REPO_URL}@{ref}",
    ]
    if subprocess.run(install_cmd).returncode != 0:
        banner(False, "시스템 라이브러리 설치 실패")
        return 1

    # 위 설치가 setuptools를 80 이상으로 끌어올리면 colcon-core(<80 요구)가 깨진다.
    # (실제로 겪은 문제: colcon build가 "error: option --uninstall not recognized"로 실패)
    step("setuptools 버전 고정 (colcon-core 호환, <80)")
    fix_cmd = [
        "sudo", sys.executable, "-m", "pip", "install",
        "--break-system-packages", "--ignore-installed", "setuptools<80,>=30.3.0",
    ]
    if subprocess.run(fix_cmd).returncode != 0:
        banner(False, "setuptools 재고정 실패")
        return 1

    banner(True, "시스템 python3.12 런타임 라이브러리 설치 완료")
    return 0


def _cleanup_venv() -> None:
    """
    .venv-infer 는 vendor 복사의 "재료"일 뿐 실행 환경이 아니라서(파일 상단 설명 참고),
    이 스크립트가 성공적으로 끝나면 더 남아 있을 이유가 없다. 다음에 이 스크립트를
    다시 돌리면 setup_repo_venv.install() 이 없는 걸 보고 알아서 새로 만든다.
    """
    if VENV_DIR.exists():
        step(f"{VENV_DIR.name} 정리 중 (vendor 복사는 이미 끝났으므로 더 필요 없음)")
        shutil.rmtree(VENV_DIR)


if __name__ == "__main__":
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    if ref is None:
        step("최신 태그 확인 중")
        ref = find_latest_tag(REPO_URL)
        if ref is None:
            raise SystemExit("최신 태그를 확인하지 못했습니다 (네트워크/Github 인증 확인 필요.)")
        step(f"최신 태그: {ref}")

    rc = install(
        REPO_URL, VENV_DIR,
        verify_hint="from src.capture.extractor import FeatureExtractor; print('OK')",
        ref=ref, system_site_packages=True,
    )
    if rc == 0:
        _vendor_copy()
        rc = _install_system_runtime_deps(ref)
    if rc == 0:
        _cleanup_venv()
    raise SystemExit(rc)
