# Last updated: 2026-07-28
'''
Signal-transport-perception을 pip으로 설치한 venv를 만든다.

ref를 인자로 주지 않으면 하드코딩된 버전이 아니라 항상 origin의 최신 version태그를 조회해서 설치
(set_infer_env와 동일)

Signal-Vision과 별도의 가상환경인 .venv-perception을 쓴다. (두 레포지토리의 의존성 충돌문제)

'''
import sys
from pathlib import Path
from setup_repo_venv import install

REPO_URL = "https://github.com/SAX-AI-TeamProject1/Signal-transport-perception.git"
VENV_DIR = Path(__file__).resolve().parent.parent / ".venv-perception"

if __name__ == "__main__":
    ref = sys.argv[1] if len(sys.argv) > 1 else None
    raise SystemExit(install(
        REPO_URL, VENV_DIR,
        verify_hint="from src.infer import load_model; print('OK')",
        ref=ref, system_site_packages=True,
    ))