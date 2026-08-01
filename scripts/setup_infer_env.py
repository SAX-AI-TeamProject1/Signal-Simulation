# Last updated: 2026-07-31
"""Signal-Vision 소스를 signal_vision 패키지로 vendor 복사하고, 런타임 라이브러리를 시스템에 깐다.

venv는 쓰지 않는다. 예전에는 .venv-infer를 만들어 거기에 Signal-Vision을 pip 설치한 뒤 그
설치본을 복사해 왔는데, 그 venv는 복사가 끝나면 바로 지워지는 일회용이었다(옛 _cleanup_venv).
즉 복사 원본 몇 MB를 얻으려고 torch/mediapipe까지 딸려오는 pip 의존성 트리를 통째로 받아
설치했다가 버리는 구조였다. 게다가 "venv 파이썬으로 camera_node를 실행하면 된다"는 설명이
남아 있었지만 그건 성립하지 않는다 — colcon이 만드는 console_scripts(ros2 run이 실행하는
것)는 venv가 활성화돼 있어도 항상 shebang이 시스템 python3.12로 박히기 때문에 venv 안의
라이브러리에는 애초에 닿지 못한다. 그래서 지금은 얕은 clone으로 소스만 가져온다.
이 스크립트 자체를 실행하는 인터프리터 버전은 상관없다.

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 v*.*.* 태그를
`git ls-remote`로 조회해서 그걸 쓴다 — 예전처럼 스크립트에 박아둔 버전이 실제
최신 fix보다 뒤처지는 걸 막기 위함 (예: mediapipe/numpy/opencv aarch64 호환성 수정).

clone한 레포의 `src/` 디렉터리를 signal_vision/signal_vision/vision_hand/ 로 그대로
복사한다(vendor). camera_node.py는 `from src...`가 아니라
`from signal_vision.vision_hand...`로 import한다 — Signal-Vision의 최상위 패키지명이
하필 `src`라서, 이 레포 자체의 src/ 디렉터리와 이름이 겹치는 걸 피하기 위함이다.
이 스크립트를 다시 돌릴 때마다 vision_hand/도 최신 태그 기준으로 통째로 교체되므로
수동으로 다시 복사할 필요가 없다.

clone한 레포 루트의 models/sign_classifier.pt(학습된 LSTM 가중치)도 함께
signal_vision/signal_vision/models/sign_classifier.pt 로 복사한다. 이 파일은 src/ 밖에
있어서 위 vendor 복사 범위에 안 들어가는데, hand_landmarker.task/pose_landmarker.task와
달리 최초 실행 시 자동 다운로드되는 경로도 없어서(구글 저장소가 아니라 이 프로젝트가
직접 학습한 가중치이므로) 여기서 복사해 주지 않으면 갱신할 방법이 없다.

복사한 파일들 내부에도 서로를 `from src.xxx import ...`로 참조하는 절대 import가 섞여
있어서(원래 패키지명이 src였으니 당연함), 그대로 두면 vision_hand 안의 파일들이 서로가
아니라 시스템/venv에 pip으로 깔린 별개의 src 패키지를 참조하게 된다 — 두 사본이 버전
드리프트로 갈라져도 아무도 못 눈치채는 상황이 생긴다. 그래서 복사 직후 vision_hand/ 안의
모든 `from src.` / `import src.` 를 `from signal_vision.vision_hand.` /
`import signal_vision.vision_hand.` 로 일괄 치환해서, vision_hand가 pip src 패키지 없이도
완전히 자기 완결적으로 동작하게 만든다.

마지막으로 torch/mediapipe/numpy/opencv-contrib-python을 **시스템 python3.12**에 설치한다.
vision_hand가 import 문 레벨에서는 자기 완결적이어도, 그 안에서 쓰는 torch/mediapipe 같은
실제 라이브러리는 시스템 python3.12에 있어야 `ros2 run signal_vision camera_node`가 동작한다
(위에 적은 shebang 이유). 이미 설치돼 있으면 건너뛰므로 여러 머신에서 반복 실행해도 안전하다.
"""
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from setup_repo_venv import find_latest_tag
from task_output import banner, step

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-Vision.git"
REPO_ROOT = Path(__file__).resolve().parent.parent
VENDOR_DEST = REPO_ROOT / "src" / "signal_vision" / "signal_vision" / "vision_hand"
MODEL_DEST = REPO_ROOT / "src" / "signal_vision" / "signal_vision" / "models" / "sign_classifier.pt"

# 복사가 성공했는지 확인할 대표 파일. camera_node/inference.py 가 실제로 import 하는 모듈이라,
# 이게 없으면 상류 레포가 구조를 바꾼 것이므로 조용히 넘어가지 않고 여기서 실패시킨다.
VENDOR_SENTINEL = Path("capture") / "extractor.py"

# 학습된 LSTM 가중치. Signal-Vision 레포에서 src/ 의 형제 디렉터리인 models/ 아래에 있어
# _vendor_copy()가 복사하는 src/ 범위 밖이므로 따로 찾아 복사해야 한다 — 안 그러면
# hand_landmarker.task/pose_landmarker.task(구글 저장소에서 최초 실행 시 자동 다운로드됨,
# extractor.py의 ensure_hand_model/ensure_pose_model 참고)와 달리 이 파일만 자동 다운로드
# 경로가 없어서 vendor 복사가 유일한 공급원인데, 그게 빠져 있으면 predict.py가 조용히
# 예전 가중치(또는 아예 없는 파일)를 계속 쓰게 된다.
MODEL_SENTINEL = Path("models") / "sign_classifier.pt"

# 줄 시작(들여쓰기 허용)의 "from src." / "import src." / "import src " 형태만 건드린다.
# 문자열/주석 안의 우연한 "src" 언급까지 건드리지 않도록 import 문 자리로 한정.
_SRC_IMPORT_RE = re.compile(r'^(\s*(?:from|import)\s+)src(\.|(?=\s))', re.MULTILINE)


def _rewrite_internal_imports() -> None:
    for py_file in VENDOR_DEST.rglob("*.py"):
        text = py_file.read_text()
        patched = _SRC_IMPORT_RE.sub(r'\1signal_vision.vision_hand\2', text)
        if patched != text:
            py_file.write_text(patched)


def _vendor_copy(ref: str) -> int:
    """지정 태그를 얕게 clone 해서 그 src/ 를 vision_hand/ 로 복사한다.

    --depth 1 인 이유: 필요한 건 그 태그 시점의 파일뿐이고 히스토리는 쓸 데가 없다.
    임시 디렉터리를 쓰므로 성공하든 실패하든 작업 트리에 clone 흔적이 남지 않는다.
    """
    with tempfile.TemporaryDirectory(prefix="signal-vision-") as tmp:
        clone_dir = Path(tmp) / "repo"
        step(f"Signal-Vision {ref} clone 중 (--depth 1)")
        clone = subprocess.run(
            ["git", "clone", "--depth", "1", "--branch", ref, REPO_URL, str(clone_dir)],
            capture_output=True, text=True,
        )
        if clone.returncode != 0:
            banner(False, "clone 실패 (네트워크/Github 인증 확인 필요)")
            print(clone.stderr, file=sys.stderr)
            return 1

        src_pkg = clone_dir / "src"
        if not (src_pkg / VENDOR_SENTINEL).is_file():
            banner(False, f"clone 은 됐지만 {src_pkg.name}/{VENDOR_SENTINEL} 가 없습니다 "
                          "— 상류 레포 구조가 바뀐 것으로 보입니다")
            return 1

        model_file = clone_dir / MODEL_SENTINEL
        if not model_file.is_file():
            banner(False, f"clone 은 됐지만 {MODEL_SENTINEL} 가 없습니다 "
                          "— 상류 레포 구조가 바뀐 것으로 보입니다")
            return 1

        step(f"vendor 복사 중 -> {VENDOR_DEST}")
        if VENDOR_DEST.exists():
            shutil.rmtree(VENDOR_DEST)
        shutil.copytree(src_pkg, VENDOR_DEST, ignore=shutil.ignore_patterns("__pycache__"))

        step(f"학습된 가중치 복사 중 -> {MODEL_DEST}")
        MODEL_DEST.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(model_file, MODEL_DEST)

    # 이 디렉터리를 ament 린터(colcon test 의 flake8) 대상에서 뺀다. 남의 레포 코드라
    # 이 리포지토리의 스타일 규칙을 강제할 대상이 아니고, 고쳐도 다음 복사 때 덮인다.
    # 위 rmtree 가 이 마커까지 지우므로 복사할 때마다 여기서 다시 만든다.
    # (ament_pep257 은 이 마커를 안 보므로 signal_vision/test/test_pep257.py 가 --exclude 로 따로 뺀다)
    (VENDOR_DEST / "AMENT_IGNORE").touch()

    _rewrite_internal_imports()
    banner(True, f"vendor 복사 완료 ({ref}) — 내부 src.* import도 signal_vision.vision_hand.*로 치환, "
                 "sign_classifier.pt도 갱신")
    return 0


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


def _cleanup_legacy_venv() -> None:
    """예전 버전이 만들어 두고 간 .venv-infer 를 지운다.

    지금은 venv를 아예 만들지 않으므로, 이전 버전의 스크립트를 돌린 적 있는 머신에는
    쓰이지 않는 수 GB짜리 디렉터리가 남아 있다. 한 번 정리해 주고 끝낸다.
    """
    legacy = REPO_ROOT / ".venv-infer"
    if legacy.exists():
        step(f"더 이상 쓰지 않는 {legacy.name} 정리 중 (이제 venv 없이 동작한다)")
        shutil.rmtree(legacy, ignore_errors=True)


if __name__ == "__main__":
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    if ref is None:
        step("최신 태그 확인 중")
        ref = find_latest_tag(REPO_URL)
        if ref is None:
            raise SystemExit("최신 태그를 확인하지 못했습니다 (네트워크/Github 인증 확인 필요.)")
        step(f"최신 태그: {ref}")

    rc = _vendor_copy(ref)
    if rc == 0:
        rc = _install_system_runtime_deps(ref)
    if rc == 0:
        _cleanup_legacy_venv()
    raise SystemExit(rc)
