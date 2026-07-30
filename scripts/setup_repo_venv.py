# Last updated: 2026-07-28
'''git+로 설치하는 외부 레포 전용 venv를 만드는 공통 로직.

setup_infer_env.py(Signal-Vision), setup_perception_env.py(Signal-transport-perception)가
공유한다 — venv 생성/최신 태그 조회/pip install 흐름은 동일하고 레포 주소·venv 이름·
설치 확인 커맨드·--system-site-packages 여부만 다르기 때문.
'''

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

from task_output import banner, step

_VERSION_TAG = re.compile(r"v(\d+)\.(\d+)\.(\d+)$")

def find_latest_tag(repo_url:str) -> str | None:
    result = subprocess.run(
        ["git","ls-remote","--tags","--refs",repo_url],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        return None

    tags = []
    for line in result.stdout.splitlines():
        name = line.rsplit("refs/tags/",1)[-1]
        m = _VERSION_TAG.search(name)
        if m:
            tags.append((tuple(int(x) for x in m.groups()),name))

    return max(tags)[1] if tags else None


def find_python312() -> list[str] | None:
    if sys.platform == "win32":
        result = subprocess.run(["py", "-3.12", "-c", "print(1)"], capture_output=True)
        return ["py", "-3.12"] if result.returncode == 0 else None
    exe = shutil.which("python3.12")
    return [exe] if exe else None


def venv_python(venv_dir:Path) -> Path:
    if sys.platform == "win32":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"

# repo_url을 git+ 형태로 venv_dir에 설치한다. 
# ref 생략 시 최신 version 태그를 자동 탐색
def install(repo_url:str, venv_dir:Path, verify_hint:str, *, ref: str | None = None, system_site_packages:bool=False)->int:
    step("python3.12 탐색 중")
    python312 = find_python312()
    if python312 is None:
        banner(False, "python3.12를 찾을 수 없습니다 - 먼저 설치하세요. ")
        return 1


    if ref is None:
        step(f"{venv_dir.name} 최신 태그 확인 중")
        ref = find_latest_tag(repo_url)
        if ref is None:
            banner(False, "최신 태그를 확인하지 못했습니다 (네트워크/Github 인증 확인 필요.)")
            return 1

        step(f"최신 태그 : {ref}")


    if not venv_python(venv_dir).exists():
        if venv_dir.exists():
            shutil.rmtree(venv_dir)
        step(f"{venv_dir.name} 생성 중 ({' '.join(python312)})")
        cmd = [*python312, "-m", "venv"]
        if system_site_packages:
            cmd.append("--system-site-packages")
        cmd.append(str(venv_dir))
        if subprocess.run(cmd).returncode !=0:
            shutil.rmtree(venv_dir,ignore_errors=True)
            banner(False, "venv 생성 실패")
            return 1


    py = str(venv_python(venv_dir))
    step("pip 업그레이드 중")
    subprocess.run([py, "-m", "pip", "install", "--upgrade", "pip"],check=True)
    step("의존성 확인/설치 중")
    dep = f"git+{repo_url}@{ref}"
    subprocess.run([py,"-m","pip","install",dep],check=True)
    step("소스 갱신 중({ref})")
    subprocess.run([py, "-m","pip","install","--force-reinstall","--no-deps",dep],check=True)

    banner(True, f'설치 완료. 확인: {py} -c "{verify_hint}"')
    return 0