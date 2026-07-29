"""
책임 1 — 하드웨어 캡처.

이 파일은 rclpy 를 import 하지 않는다. 일부러 그렇다.
    - ROS 없이 단독으로 돌려볼 수 있다(카메라만 따로 검증할 때).
    - 나중에 파일/시뮬레이션 영상 소스로 갈아끼울 때 이 파일만 바꾸면 된다.

import 목록에 cv2 만 있는 것이 그 사실의 증거다. 이 줄이 늘어나면(특히 rclpy 가
들어오면) 분리가 무너진 것이니 되돌릴 것.
"""

import cv2

# OpenCV 캡처 백엔드 이름 → cv2 상수.
#
# V4L2 를 하드코딩하면 Linux 밖에서 장치가 열리지 않는다. 실배포는 Linux 라
# 기본값은 v4l2 로 두되, 개발 머신에 맞춰 바꿀 수 있게 파라미터로 뺐다.
CAPTURE_BACKENDS = {
    'v4l2': cv2.CAP_V4L2,                   # Linux (기본, 실제 배포 환경)
    'any': cv2.CAP_ANY,                     # OpenCV 가 알아서 고름
    'avfoundation': cv2.CAP_AVFOUNDATION,   # macOS
    'dshow': cv2.CAP_DSHOW,                 # Windows
}


class FrameSource:
    """
    웹캠 장치 하나를 열고 프레임을 읽어 준다. cv2.VideoCapture 의 얇은 래퍼.

    로그를 직접 찍지 않는다. 상태는 프로퍼티로 노출만 하고, 무엇을 경고할지는
    호출자(CameraNode)가 정한다. 로거를 주입받으면 다시 ROS 에 묶이기 때문이다.
    """

    def __init__(self, device_id, width, height, fps, backend='v4l2'):
        """
        장치를 열고 해상도/FPS 를 설정한다. 열지 못하면 RuntimeError.

        인자:
            device_id: /dev/video<N> 의 N
            width, height: 요청 해상도(카메라가 거절할 수 있다 → actual_size 로 확인)
            fps: 요청 프레임레이트(마찬가지로 카메라가 거절할 수 있다)
            backend: CAPTURE_BACKENDS 의 키 문자열
        """
        if width <= 0 or height <= 0:
            raise ValueError(f'해상도는 0 보다 커야 합니다 (받은 값: {width}x{height})')
        if backend not in CAPTURE_BACKENDS: # keys 대상으로 확인
            raise ValueError(
                f'알 수 없는 캡처 백엔드: {backend!r} (가능: {list(CAPTURE_BACKENDS)})')

        self._device_id = device_id
        self._requested_size = (width, height)

        # 백엔드를 명시하는 이유: 지정하지 않으면 OpenCV 가 다른 백엔드를 잡아
        # 해상도/FPS 설정이 조용히 무시되는 경우가 있다.
        self._cap = cv2.VideoCapture(device_id, CAPTURE_BACKENDS[backend])
        if not self._cap.isOpened():
            raise RuntimeError(
                f'웹캠을 열지 못했습니다: /dev/video{device_id} (backend={backend}) — '
                '장치 존재 여부(ls /dev/video*)와 권한(video 그룹)을 확인하세요.')

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        # 드라이버 내부 버퍼를 1장으로 제한한다. 이게 없으면:
        #   카메라가 30fps 로 채우는데 우리가 20Hz 로 읽으면 초당 10장이 버퍼에 쌓이고,
        #   read() 는 "가장 오래된" 프레임부터 돌려준다 → 시간이 갈수록 화면이 밀린다.
        # 애플리케이션 큐(LatestFrameQueue)를 1로 줄여도 이걸 안 하면
        # 드라이버 단에서 지연이 남는다.
        # (백엔드가 지원하지 않으면 조용히 무시된다 — 설정해서 손해 볼 일은 없다)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    @property
    def device_id(self):
        """열려 있는 장치 번호."""
        return self._device_id

    @property
    def requested_size(self):
        """요청했던 (width, height)."""
        return self._requested_size

    @property
    def actual_size(self):
        """카메라가 실제로 적용한 (width, height). 요청과 다를 수 있다."""
        if self._cap is None:
            return (0, 0)
        return (int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    @property
    def is_open(self):
        """아직 장치를 들고 있으면 True. release() 후에는 False."""
        return self._cap is not None

    def read(self):
        """
        프레임 한 장을 읽는다.

        반환:
            numpy.ndarray (H, W, 3) uint8 BGR — 성공
            None — 실패했거나 이미 release() 된 상태

        주의: 이 호출은 블로킹이다(다음 프레임이 올 때까지 최대 1/fps 만큼 멈춘다).
              그래서 호출자는 이걸 실행기 스레드에서 부르되 그 뒤에 무거운 일을
              얹지 않아야 한다.
        """
        if self._cap is None:
            return None
        ok, frame = self._cap.read()
        # cv2 의 read() 는 호출마다 새 배열을 반환한다. 그래서 이 프레임을 워커에게
        # 넘겨도 다음 read() 가 덮어쓰지 않는다(별도 복사 불필요).
        if ok:
            return frame
        return None

    def release(self):
        """장치를 반납한다. 여러 번 불러도 안전하다."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
