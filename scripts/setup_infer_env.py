# Last updated: 2026-07-27
"""Signal-Vision을 pip으로 설치한 venv를 만든다 (rclpy 연동은 아직 없음 — 순수 설치 확인용).
ROS와 무관한 작업이라 bash 대신 순수 파이썬으로 작성 — Windows/macOS/Linux 어디서든 동일하게 동작한다.
이 스크립트 자체를 실행하는 인터프리터 버전은 상관없다 (python3.12는 내부에서 직접 찾는다).

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 v*.*.* 태그를
`git ls-remote`로 조회해서 그걸 설치한다 — 예전처럼 스크립트에 박아둔 버전이 실제
최신 fix보다 뒤처지는 걸 막기 위함 (예: mediapipe/numpy/opencv aarch64 호환성 수정).
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

from task_output import banner, step

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-Vision.git"
REPO = f"git+{REPO_URL}"
VENV_DIR = Path(__file__).resolve().parent.parent / ".venv-infer"

_VERSION_TAG = re.compile(r"v(\d+)\.(\d+)\.(\d+)$")


def find_latest_tag(repo_url: str) -> str | None:
    """origin의 v<major>.<minor>.<patch> 태그 중 semver로 가장 높은 것을 고른다."""
    result = subprocess.run(
        ["git", "ls-remote", "--tags", "--refs", repo_url],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None

    tags = []
    for line in result.stdout.splitlines():
        name = line.rsplit("refs/tags/", 1)[-1]
        m = _VERSION_TAG.search(name)
        if m:
            tags.append((tuple(int(x) for x in m.groups()), name))
    if not tags:
        return None
    return max(tags)[1]


def find_python312() -> list[str] | None:
    """Signal-Vision의 src/tools/setup_venv.py와 동일한 탐색 방식 (같은 3.12 고정 요구사항)."""
    if sys.platform == "win32":
        result = subprocess.run(["py", "-3.12", "-c", "print(1)"], capture_output=True)
        return ["py", "-3.12"] if result.returncode == 0 else None
    exe = shutil.which("python3.12")
    return [exe] if exe else None


def venv_python(venv_dir: Path) -> Path:
    # Windows venv는 Scripts\python.exe, macOS/Linux venv는 bin/python — OS별 레이아웃이 다르다.
    if sys.platform == "win32":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"


def main() -> int:
    step("python3.12 탐색 중")
    python312 = find_python312()
    if python312 is None:
        banner(False, "python3.12를 찾을 수 없습니다 — Signal-Vision은 3.12 고정입니다. 먼저 설치하세요.")
        return 1

    ref = sys.argv[1] if len(sys.argv) > 1 else None
    if ref is None:
        step("Signal-Vision 최신 태그 확인 중")
        ref = find_latest_tag(REPO_URL)
        if ref is None:
            banner(
                False,
                "Signal-Vision의 최신 태그를 확인하지 못했습니다 (네트워크/GitHub 인증 확인).\n"
                "ref를 직접 지정해서 재시도하세요: python3 scripts/setup_infer_env.py <ref>",
            )
            return 1
        step(f"최신 태그: {ref}")

    # venv 생성이 중간에 실패하면(예: ensurepip 누락) 깨진 디렉터리가 남아
    # 다음 실행에서 "이미 존재함"으로 오인되므로, 실제 파이썬 실행 파일 유무로 판단한다.
    if not venv_python(VENV_DIR).exists():
        if VENV_DIR.exists():
            shutil.rmtree(VENV_DIR)
        step(f"{VENV_DIR.name} 생성 중 ({' '.join(python312)})")
        result = subprocess.run([*python312, "-m", "venv", str(VENV_DIR)])
        if result.returncode != 0:
            shutil.rmtree(VENV_DIR, ignore_errors=True)
            banner(
                False,
                "venv 생성 실패 (ensurepip 누락 등). Debian/Ubuntu에서는 아래 명령 실행 후\n"
                "이 태스크를 다시 실행하세요:\n\n"
                "  sudo apt install -y python3.12-venv",
            )
            return 1

    py = str(venv_python(VENV_DIR))
    step("pip 업그레이드 중")
    subprocess.run([py, "-m", "pip", "install", "--upgrade", "pip"], check=True)
    step("Signal-Vision 의존성 확인/설치 중")
    subprocess.run([py, "-m", "pip", "install", f"{REPO}@{ref}"], check=True)
    # pyproject.toml의 version이 태그마다 안 올라가서, pip이 "이미 설치됨"으로 보고
    # ref가 바뀐 소스 갱신을 건너뛸 수 있다 — signal-vision 소스만 강제로 다시 받는다.
    # (의존성은 위에서 이미 확인됐으니 --no-deps로 무거운 재설치는 피한다)
    step(f"Signal-Vision 소스 갱신 중 ({ref})")
    subprocess.run([py, "-m", "pip", "install", "--force-reinstall", "--no-deps", f"{REPO}@{ref}"], check=True)

    banner(True, f'설치 완료. 확인: {py} -c "from src.capture.extractor import FeatureExtractor; print(\'OK\')"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
