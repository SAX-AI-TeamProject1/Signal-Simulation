# Last updated: 2026-07-31
'''Signal-transport-perception을 pip으로 설치한 venv를 만들고, track_follow.py를
auto_drive/patrol/ 로 vendor 복사한 뒤 시스템 python3.12에 런타임 의존성을 설치하고,
Kaggle에서 학습된 obstacle 탐지 모델(best.pt)까지 받아온다.

(2026-08-01: robot_control 패키지가 auto_drive/signal_vision/knavi_bringup 셋으로
분리되면서, 이 스크립트가 vendor 복사·모델 배치에 쓰는 경로도 auto_drive로 옮겼다.)

setup_infer_env.py(Signal-Vision)와 같은 패턴이다 — venv는 vendor 복사의 "재료"일 뿐
실행 환경이 아니므로 끝나면 지운다(colcon이 빌드하는 console_scripts는 항상 시스템
python3.12로 실행되기 때문에 venv가 남아 있어도 어차피 못 씀. setup_infer_env.py 헤더
주석에 이 근거가 자세히 있다).

Signal-Vision과 별도의 가상환경인 .venv-perception을 쓰는 이유: 두 레포지토리의
의존성 충돌 — Signal-transport-perception(ultralytics)은 opencv-python을, mediapipe는
opencv-contrib-python을 요구한다. 둘 다 같은 cv2/ 자리에 설치되는 별개 pip 패키지라
한 환경에 같이 못 둔다(run_camera_node.sh 주석 참고). 그래서 벤더링용 venv도 분리하고,
아래 시스템 설치 단계에서도 opencv-contrib-python을 마지막에 강제로 복원한다.

Kaggle 인증(KAGGLE_USERNAME/KAGGLE_KEY)은 이 레포 루트의 .env에서 읽는다 — 모델을
실제로 쓰는 건 Signal-Simulation(이 레포)의 ROS 노드이므로, 키 입력도 모델 파일도
Signal-transport-perception이 아니라 여기 있어야 한다. 읽는 방식은
Signal-transport-perception/tools/fetch_model.py와 동일하게 python-dotenv의
load_dotenv()를 쓴다 — 같은 걸 두 레포가 서로 다른 방식(한쪽은 라이브러리, 한쪽은
직접 파싱)으로 읽으면 나중에 .env 문법이 조금만 달라져도 한쪽만 깨진다.
python-dotenv는 순수 파이썬이라(C 확장 없음) opencv-python/opencv-contrib-python
같은 충돌 위험이 없어 시스템 python3.12에 설치해도 안전하다. .env가 비어 있으면
모델 받기 단계만 건너뛰고(vendor 복사는 이미 끝난 상태이므로) 나머지는 정상 종료한다.

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 version태그를 조회해서 설치
(set_infer_env와 동일)
'''
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from setup_repo_venv import find_latest_tag, install, venv_python
from task_output import banner, step

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-transport-perception.git"
KAGGLE_KERNEL = "heojaeseong/yolo-percep"  # fetch_model.py(Signal-transport-perception)와 동일한 커널
REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv-perception"
VENDOR_DEST = (REPO_ROOT / "src" / "auto_drive" / "auto_drive"
               / "patrol" / "track_follow.py")
MODEL_DEST = REPO_ROOT / "src" / "auto_drive" / "models" / "obstacle_detector.pt"
DOTENV_PATH = REPO_ROOT / ".env"


def _vendor_copy() -> None:
    """venv에 pip으로 깔린 Signal-transport-perception의 src/track_follow.py 한 파일을
    auto_drive/patrol/track_follow.py로 복사한다(track_marker_infer.py와 같은 위치,
    같은 "벤더 복사본" 패턴).

    vision_hand처럼 패키지 전체를 복사하지 않고 파일 하나만 복사하는 이유: 인계 문서
    (docu/track-follow-integration-guide.md) 2.1에 이 함수가 estimate_track() 하나짜리
    순수 함수 모듈로 설명돼 있어서다. 다만 이 레포에서는 실제 src/track_follow.py 내용을
    본 적이 없으므로, 만약 그 파일이 내부적으로 `from src.xxx import ...`처럼 형제
    모듈을 참조한다면 아래 임포트 검증에서 바로 실패해 원인이 드러난다 — 그 경우 이
    함수를 vision_hand 방식(패키지 전체 복사 + import 경로 일괄 치환)으로 바꿔야 한다.
    """
    py = str(venv_python(VENV_DIR))
    result = subprocess.run(
        [py, "-c",
         "import src.track_follow, pathlib; print(pathlib.Path(src.track_follow.__file__))"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise SystemExit(
            f"vendor 복사 실패: venv에서 src.track_follow를 찾을 수 없습니다\n{result.stderr}")

    src_file = Path(result.stdout.strip())
    VENDOR_DEST.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(src_file, VENDOR_DEST)
    print(f"vendor 복사 완료: {src_file} -> {VENDOR_DEST}")

    # 복사만으로 끝내지 않고 실제로 auto_drive 패키지 경로에서 import가 되는지 바로
    # 확인한다 — 형제 모듈 참조가 있으면 여기서 ModuleNotFoundError로 즉시 드러난다.
    verify = subprocess.run(
        [sys.executable, "-c",
         "import importlib.util, sys; "
         f"spec = importlib.util.spec_from_file_location('track_follow', r'{VENDOR_DEST}'); "
         "mod = importlib.util.module_from_spec(spec); "
         "spec.loader.exec_module(mod); "
         "assert hasattr(mod, 'estimate_track'), 'estimate_track 함수가 없습니다'; "
         "print('OK')"],
        capture_output=True, text=True,
    )
    if verify.returncode != 0:
        banner(False,
               "vendor 복사된 track_follow.py를 단독으로 import할 수 없습니다 — "
               "형제 모듈을 참조하는 것으로 보입니다. vision_hand처럼 패키지 전체를 "
               "복사하는 방식으로 바꿔야 합니다.")
        print(verify.stderr, file=sys.stderr)
        raise SystemExit(1)


def _ensure_python_dotenv_installed() -> int:
    """python-dotenv를 시스템 python3.12에 설치한다. 순수 파이썬 패키지라 시스템 전역
    site-packages(root 소유)에 안 넣어도 되므로, ultralytics/opencv와 달리 sudo 없이
    --user로 설치한다(비밀번호 입력이 안 되는 비대화형 환경에서도 그냥 동작하게)."""
    check = subprocess.run([sys.executable, "-c", "import dotenv"], capture_output=True)
    if check.returncode == 0:
        return 0
    step("시스템 python3.12에 python-dotenv 없음 — 설치 중 (--user, sudo 불필요)")
    result = subprocess.run([
        sys.executable, "-m", "pip", "install",
        "--break-system-packages", "--user", "--ignore-installed", "python-dotenv",
    ])
    if result.returncode != 0:
        banner(False, "python-dotenv 설치 실패")
        return 1
    # --user site-packages 디렉터리가 인터프리터 시작 시점에 없었으면 sys.path에
    # 안 들어가 있어서, 방금 설치했어도 같은 프로세스에서는 import가 안 된다.
    # user site 경로를 지금 sys.path에 반영해 재실행 없이 바로 쓸 수 있게 한다.
    import site
    import importlib
    user_site = site.getusersitepackages()
    if user_site not in sys.path:
        site.addsitedir(user_site)
    importlib.invalidate_caches()
    return 0


def _fetch_model_weights() -> int:
    """이 레포 루트 .env의 KAGGLE_USERNAME/KAGGLE_KEY로 Kaggle 커널의 최근 완료된
    실행 결과에서 obstacle 탐지 모델(best.pt)을 받아 models/obstacle_detector.pt로 배치한다.

    Signal-transport-perception/tools/fetch_model.py와 하는 일은 같지만, 그 스크립트는
    그 레포 안에서만 쓸 수 있고(자기 venv의 kaggle CLI, 자기 data/ 경로) 정작 모델을 쓰는 건
    이 레포이므로 여기서 다시 구현한다. kaggle CLI는 .venv-perception에 pip으로 설치된
    걸 그대로 쓴다(Signal-transport-perception의 pyproject.toml에 kaggle==2.2.4가
    의존성으로 선언돼 있어 install() 단계에서 이미 같이 깔림).

    .env를 읽는 방식(load_dotenv)은 fetch_model.py와 동일하게 맞춘다(파일 상단 설명 참고).

    인증 방법은 legacy(KAGGLE_USERNAME/KAGGLE_KEY, .env)와 access_token
    (~/.kaggle/access_token) 둘 다 지원한다 — kaggle 패키지의 authenticate()가
    access_token을 1순위로 자동으로 먼저 시도하므로 여기서는 둘 중 하나라도
    설정돼 있으면 그냥 진행시키고, 실제 인증 성공/실패 판단은 kaggle CLI 자체에 맡긴다.
    """
    if _ensure_python_dotenv_installed() != 0:
        return 1

    from dotenv import load_dotenv
    load_dotenv(DOTENV_PATH)
    kaggle_username = os.environ.get("KAGGLE_USERNAME", "")
    kaggle_key = os.environ.get("KAGGLE_KEY", "")
    has_legacy_creds = bool(kaggle_username and kaggle_key)
    access_token_path = Path.home() / ".kaggle" / "access_token"
    has_access_token = access_token_path.is_file() and access_token_path.stat().st_size > 0
    if not has_legacy_creds and not has_access_token:
        step(f"{DOTENV_PATH.name}의 KAGGLE_USERNAME/KAGGLE_KEY도, "
             f"{access_token_path}도 비어 있음 — 모델 다운로드는 건너뜀 "
             "(벤더 복사는 이미 끝남, 둘 중 하나를 채운 뒤 다시 실행하면 됨)")
        return 0

    step("Kaggle에서 obstacle 탐지 모델(best.pt) 받는 중")
    kaggle_bin = str(venv_python(VENV_DIR).parent / "kaggle")
    # load_dotenv()가 이미 os.environ에 KAGGLE_USERNAME/KAGGLE_KEY를 채웠으므로
    # fetch_model.py와 동일하게 os.environ을 그대로 넘긴다.
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
    tmp_dir = Path(tempfile.mkdtemp())
    result = subprocess.run(
        [kaggle_bin, "kernels", "output", KAGGLE_KERNEL, "-p", str(tmp_dir)], env=env)
    if result.returncode != 0:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        banner(False, "Kaggle 모델 다운로드 실패 — KAGGLE_USERNAME/KAGGLE_KEY 값을 확인하세요.")
        return 1

    src_weights = tmp_dir / "runs" / "train" / "weights" / "best.pt"
    if not src_weights.is_file():
        shutil.rmtree(tmp_dir, ignore_errors=True)
        banner(False, f"다운로드했지만 {src_weights}를 찾을 수 없습니다 (커널 출력 구조 확인 필요).")
        return 1

    MODEL_DEST.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(src_weights, MODEL_DEST)
    shutil.rmtree(tmp_dir, ignore_errors=True)
    print(f"모델 다운로드 완료: {MODEL_DEST}")
    return 0


def _install_system_runtime_deps() -> int:
    """
    ultralytics를 시스템 python3.12에 설치한다.

    ultralytics는 의존성으로 opencv-python을 끌어오는데, 시스템에는 이미 mediapipe용
    opencv-contrib-python이 깔려 있다(setup_infer_env.py가 설치). 둘 다 같은 cv2/
    자리에 설치되는 별개 pip 패키지라, ultralytics 설치 과정에서 opencv-contrib-python이
    opencv-python으로 덮어써질 수 있다.

    ultralytics의 정확한 의존성 목록을 여기 하드코딩하지 않는 이유: ultralytics
    버전이 올라가면 그 목록도 바뀌어서 하드코딩이 조용히 낡는다. 대신 pip이 알아서
    다 설치하게 둔 뒤, 마지막에 opencv-contrib-python만 --force-reinstall로 다시
    얹어 원복한다 — contrib가 opencv-python의 상위집합이라 이렇게 되돌려도
    ultralytics가 쓰는 cv2 API는 그대로 동작한다.
    """
    check = subprocess.run([sys.executable, "-c", "import ultralytics"],
                           capture_output=True)
    if check.returncode == 0:
        step("시스템 python3.12에 ultralytics 이미 설치됨 — 건너뜀")
        return 0

    step("시스템 python3.12에 ultralytics 없음 — 설치 시작 (sudo 비밀번호 필요할 수 있음)")

    if subprocess.run([sys.executable, "-m", "pip", "--version"],
                      capture_output=True).returncode != 0:
        step("시스템 pip 없음 — apt install python3-pip")
        if subprocess.run(["sudo", "apt", "install", "-y", "python3-pip"]).returncode != 0:
            banner(False, "python3-pip 설치 실패")
            return 1

    step("ultralytics 설치 중 (수 분 소요)")
    install_cmd = [
        "sudo", sys.executable, "-m", "pip", "install",
        "--break-system-packages", "--ignore-installed", "ultralytics",
    ]
    if subprocess.run(install_cmd).returncode != 0:
        banner(False, "ultralytics 설치 실패")
        return 1

    step("opencv-contrib-python 복원 중 (ultralytics가 끌어온 opencv-python 덮어쓰기)")
    restore_cmd = [
        "sudo", sys.executable, "-m", "pip", "install",
        "--break-system-packages", "--force-reinstall", "--no-deps",
        "opencv-contrib-python",
    ]
    if subprocess.run(restore_cmd).returncode != 0:
        banner(False, "opencv-contrib-python 복원 실패")
        return 1

    # 위 설치가 setuptools를 80 이상으로 끌어올리면 colcon-core(<80 요구)가 깨진다
    # (setup_infer_env.py에서 실제로 겪은 문제와 동일).
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
    """.venv-perception은 vendor 복사의 재료일 뿐 실행 환경이 아니므로(파일 상단 설명
    참고), 이 스크립트가 성공적으로 끝나면 더 남아 있을 이유가 없다."""
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
        step(f"최신 태그 : {ref}")

    rc = install(
        REPO_URL, VENV_DIR,
        verify_hint="from src.track_follow import estimate_track; print('OK')",
        ref=ref, system_site_packages=True,
    )
    if rc == 0:
        _vendor_copy()
        rc = _fetch_model_weights()
    if rc == 0:
        rc = _install_system_runtime_deps()
    if rc == 0:
        _cleanup_venv()
    raise SystemExit(rc)
