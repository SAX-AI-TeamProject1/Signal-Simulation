# Last updated: 2026-07-27
"""Signal-Vision을 pip으로 설치한 venv를 만든다 (rclpy 연동은 아직 없음 — 순수 설치 확인용).
ROS와 무관한 작업이라 bash 대신 순수 파이썬으로 작성 — Windows/macOS/Linux 어디서든 동일하게 동작한다.
이 스크립트 자체를 실행하는 인터프리터 버전은 상관없다 (python3.12는 내부에서 직접 찾는다).
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

REPO = "git+https://github.com/SAX-AI-TeamProject1/Signal-Vision.git"
DEFAULT_REF = "v1.0.1"
VENV_DIR = Path(__file__).resolve().parent.parent / ".venv-infer"


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


def main() -> None:
    python312 = find_python312()
    if python312 is None:
        print("[setup] python3.12를 찾을 수 없습니다 — Signal-Vision은 3.12 고정입니다. 먼저 설치하세요.", file=sys.stderr)
        sys.exit(1)

    ref = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_REF

    if not VENV_DIR.exists():
        print(f"[setup] {VENV_DIR.name} 생성 ({' '.join(python312)})")
        subprocess.run([*python312, "-m", "venv", str(VENV_DIR)], check=True)

    py = str(venv_python(VENV_DIR))
    subprocess.run([py, "-m", "pip", "install", "--upgrade", "pip"], check=True)
    subprocess.run([py, "-m", "pip", "install", f"{REPO}@{ref}"], check=True)

    print(f'[setup] 완료. 확인: {py} -c "from src.capture.extractor import FeatureExtractor; print(\'OK\')"')


if __name__ == "__main__":
    main()
