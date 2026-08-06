#!/usr/bin/env python3
"""웹캠 원본 프레임 한 장을 그대로 저장하고, 화질 지표를 전부 찍는다.

왜 필요한가:
    화질 임계값을 HUD 스크린샷으로 맞추려다 계속 어긋났다. 창이 640x480 프레임을
    더 작게(그리고 세로로 찌그러뜨려) 그리기 때문에, 스크린샷을 잘라 재면 그 축소가
    흐림을 더해 실제보다 큰 값이 나온다 — 같은 순간에 노드는 절0.41, 스크린샷 측정은
    0.619 였다. 임계값은 **원본 프레임 기준**으로 정해야 한다.

사용법:
    수신호 주는 거리에서, 판정하고 싶은 상태 그대로 두고 실행한다.
        python3 scripts/capture_lens_sample.py --tag normal      # 정상일 때
        python3 scripts/capture_lens_sample.py --tag blurry      # 뿌옇게 만든 뒤
    카메라 번호가 다르면 --camera N (7번 태스크에서 쓰는 번호와 같아야 한다).

    **camera_node 가 그 카메라를 잡고 있으면 열리지 않는다.** 7번 태스크를 끄고 실행할 것.

결과:
    scratch/lens_<tag>.png 로 원본 프레임이 저장되고, 지표가 화면에 찍힌다.
    두 상태의 파일과 출력을 비교하면 임계값을 실측으로 확정할 수 있다.
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'signal_vision'))

import cv2                                                            # noqa: E402

import numpy as np                                                    # noqa: E402

from signal_vision.vision_hand.capture.extractor import (             # noqa: E402
    GLARE_LEVEL,
    LOW_LIGHT_P95,
    LensHealthMonitor,
    STRUCTURE_ABS_MIN,
    _sharpness_score,
    open_camera,
)


def capture(camera: int, warmup: int, width: int, height: int):
    """웹캠을 열어 워밍업 프레임을 버리고 마지막 한 장을 돌려준다.

    camera_node 와 같은 해상도를 요청한다 — 해상도가 다르면 지표도 달라지므로,
    여기서 잰 값이 노드에서 나오는 값과 같아야 의미가 있다.
    """
    cap = open_camera(camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    frame = None
    try:
        for _ in range(warmup):
            ok, latest = cap.read()
            if ok:
                frame = latest
    finally:
        cap.release()
    if frame is None:
        raise RuntimeError('웹캠에서 프레임을 한 장도 받지 못했습니다. '
                           '7번 태스크가 카메라를 잡고 있지 않은지 확인하세요.')
    return frame


def report(frame) -> None:
    """원본 프레임 하나에 대해 판정에 쓰이는 지표를 전부 출력한다."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    sharp = _sharpness_score(gray)
    structure = _sharpness_score(small)
    # 잡음 추정: 원본에서 중간값 필터를 뺀 나머지 = 화소끼리 상관 없는 미세 성분.
    # 이 값이 크면 고주파의 정체가 디테일이 아니라 센서 노이즈라는 뜻이다.
    noise = float(np.std(gray.astype(np.float32) - cv2.medianBlur(gray, 3).astype(np.float32)))
    p5, p50, p95 = np.percentile(gray, [5, 50, 95])

    h, w = gray.shape
    print(f'  해상도        {w}x{h}')
    print(f'  선명도        {sharp:.1f}')
    print(f'  구조          {structure:.1f}   (절대 판정선 {STRUCTURE_ABS_MIN} 미만이면 뭉개짐)')
    print(f'  잡음비        {structure / max(sharp, 1e-6):.2f}   (기준치 대비로만 판정)')
    print(f'  밝기 p5/p50/p95    {p5:.0f} / {p50:.0f} / {p95:.0f}   '
          f'(저조도 기준 p95 < {LOW_LIGHT_P95})')
    print(f'  포화 비율     {np.mean(gray >= GLARE_LEVEL) * 100:.1f}%')
    print(f'  대비(표준편차) {gray.std():.1f}')
    print(f'  잡음 추정     {noise:.2f}')


def main() -> None:
    """프레임을 잡아 저장하고 지표를 찍는다."""
    parser = argparse.ArgumentParser(description='웹캠 원본 프레임 + 화질 지표')
    parser.add_argument('--camera', type=int, default=0, help='/dev/video<N> 의 N')
    parser.add_argument('--tag', default='sample', help='저장 파일 이름에 붙일 꼬리표')
    parser.add_argument('--warmup', type=int, default=30,
                        help='버릴 워밍업 프레임 수 (노출·초점이 자리 잡기를 기다림)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()

    frame = capture(args.camera, args.warmup, args.width, args.height)
    out_dir = REPO_ROOT / 'scratch'
    out_dir.mkdir(exist_ok=True)
    out = out_dir / f'lens_{args.tag}.png'
    cv2.imwrite(str(out), frame)   # 원본 그대로 (PNG = 무손실)
    print(f'저장: {out}\n')
    report(frame)

    # 노드와 같은 판정기에 그대로 넣어 보면, 이 한 장이 이상으로 잡히는지 알 수 있다.
    # (기준치가 없어도 도는 절대 축만 반응한다 — 상대 축은 보정이 필요하다)
    monitor = LensHealthMonitor()
    monitor.update(frame)
    print(f'\n  절대 축 판정  {monitor._absolute_fault() or "이상 없음"}')


if __name__ == '__main__':
    main()
