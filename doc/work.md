# 작업 기록

## 2026-08-05 — 시뮬레이터 EMERGENCY(렌즈 블러) 미표시 수정

### 배경
Signal-Vision에는 렌즈 블러를 판정해 EMERGENCY UI를 띄우는 기능이 있으나,
그 코드를 벤더링해 쓰는 Signal-Simulation의 camera_node 경로에서는 EMERGENCY가
전혀 뜨지 않았다.

### 원인
- `camera_node/inference.py`가 `LensHealthMonitor`(블러 판정기)를 아예 만들지 않고,
  `SignalStabilizer`에 `is_blurred=False`를 하드코딩해 넘기고 있었다.
- 독립 실행 경로(`vision_hand/inference/function.py`)에는 있던 블러 배선이
  camera_node로 옮겨질 때 누락된 것.

### 변경 내용

1. **camera_node에 블러 판정 배선 추가** — `camera_node/inference.py`
   - `LensHealthMonitor` import 및 인스턴스 생성(추론 워커 전용).
   - `_infer_impl`에서 매 프레임 `is_blurred = self._lens_monitor.update(frame)`를
     계산해 `mark_person_absent`/`stabilizer.update`에 전달(하드코딩 `False` 제거).
   - 블러 판정은 윈도우 충족 여부와 무관하게 매 프레임 실행 — 초기
     `BLUR_CALIBRATION_FRAMES` 동안 정상 기준치를 잡아야 하기 때문.

2. **EMERGENCY 트리거 정책 변경** — `vision_hand/inference/predict.py`
   - 기존: "렌즈 블러 AND 인식 저하"가 겹치고, 이미 확정된 신호가 있어야만 EMERGENCY.
     → 실제로는 카메라를 완전히 상실했을 때만 떴다.
   - 변경: **렌즈 블러 단독으로 EMERGENCY + 즉시정지**. 확정 신호 유무·사람 유무와
     무관하게, `is_blurred`가 True면 최우선으로 즉시정지 처리하고 나머지 안정화
     로직은 건너뛴다. 화면이 다시 선명해지면 자동 해제.
   - 관절만 못 잡는 경우(이미지 선명)는 정상 운용으로 두어 오탐을 막는다.
   - `_reset_to_unknown`에서 emergency 관리를 분리 — emergency는 오직 `is_blurred`로만
     결정하도록 호출부에서 세팅.

3. **setup_perception_env.py — python-dotenv 설치 직후 import 실패 수정**
   - `--user` 설치 직후, 인터프리터 시작 시점에 user site-packages 경로가
     `sys.path`에 없어 같은 프로세스에서 `import dotenv`가 실패하던 문제.
   - 설치 후 `site.addsitedir()` + `importlib.invalidate_caches()`로 경로를 반영해
     재실행 없이 한 번에 통과하도록 수정.

### 반영 방법
Python 소스만 변경했고 `--symlink-install`이므로 재빌드 없이 노드 재시작만으로 반영된다.

### 남은 튜닝 포인트 (`vision_hand/capture/extractor.py`)
- `BLUR_CALIBRATION_FRAMES = 30`: 시작 시 화면이 선명해야 정상 기준치가 잡힌다.
- `BLUR_RATIO_DROP = 0.5`: 선명도가 기준치 절반 밑으로 떨어져야 블러로 판정.
  약한 초점흐림까지 잡으려면 `0.6~0.7`로 올려 감도를 높일 수 있다.
