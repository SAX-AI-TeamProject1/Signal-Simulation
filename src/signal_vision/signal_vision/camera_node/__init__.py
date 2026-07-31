"""
camera_node 를 책임별 파일로 쪼갠 패키지.

camera_node_refactory.py 는 네 가지 책임을 "클래스"로 나눴지만 여전히 한 파일이었다.
이 패키지는 그 클래스들을 "파일"로까지 나눈 것이다. 내용은 같고 배치만 다르다.

왜 파일까지 나누는가:
    - 파일 하나가 800줄이면 목차 없이는 원하는 곳을 못 찾는다. 파일 이름이 목차가 된다.
    - git blame / diff 가 책임 단위로 갈린다. 모델을 고친 커밋과 카메라를 고친 커밋이
      같은 파일을 건드리지 않으므로, 두 사람이 동시에 작업해도 충돌하지 않는다.
    - import 문이 의존 관계를 드러낸다. frame_source.py 가 rclpy 를 import 하지
      않는다는 사실이 파일 맨 위에서 바로 보인다.

┌── 어느 파일에 무엇이 있는지 ────────────────────────────────────────────────┐
│                                                                             │
│  labels.py             LABEL_MOTION, LABELS           공용 상수             │
│                                                                             │
│  frame_source.py       FrameSource                    1. 하드웨어 캡처      │
│                        CAPTURE_BACKENDS                  (rclpy 안 씀)      │
│                                                                             │
│  image_publisher.py    ImagePublisher                 2. 이미지 발행        │
│                                                                             │
│  inference.py          InferenceBase                  3. 추론 인터페이스    │
│                        GestureInference                  (rclpy 안 씀)      │
│                                                                             │
│  command_publisher.py  GestureCommandPublisher        4. 라벨 → Twist       │
│                                                                             │
│  shutdown.py           is_shutting_down()             배관: 종료 판단       │
│  swap_frame.py         LatestFrameBuffer              배관: 한 칸 버퍼      │
│                                                                             │
│  node.py               CameraNode, main()             조립자                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

의존 방향 (위가 아래를 쓴다. 화살표가 거꾸로 가는 곳은 없다 = 순환 import 없음):

    node.py
      ├─▶ frame_source.py
      ├─▶ image_publisher.py ──┐
      ├─▶ inference.py         ├─▶ shutdown.py
      ├─▶ command_publisher.py ┘        └─▶ labels.py
      ├─▶ swap_frame.py
      └─▶ signal_vision.metrics (이 패키지 안, camera_node 폴더 밖)

실행:
    ros2 run signal_vision camera_node
    ros2 run signal_vision camera_node --ros-args -p device_id:=1

이 패키지를 직접 import 해서 쓸 때는 각 모듈에서 가져온다. 여기서 재수출하지 않는
이유는, 이 __init__.py 가 cv2 와 rclpy 를 전부 끌어오게 되어 "ROS 없이 FrameSource 만
테스트한다"는 분리의 목적이 무너지기 때문이다:

    from signal_vision.camera_node.frame_source import FrameSource
    from signal_vision.camera_node.labels import LABELS
"""
