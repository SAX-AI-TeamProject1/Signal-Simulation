"""
수신호 라벨의 단일 출처(single source of truth).

왜 별도 파일인가:
    추론(inference.py)은 라벨을 "만들고", 제어 매핑(command_publisher.py)은
    라벨을 "소비한다". 두 모듈이 같은 정의를 봐야 하는데, 어느 한쪽에 두면
    다른 쪽이 그쪽을 import 해야 한다. 그러면 "추론이 제어를 안다" 같은
    엉뚱한 의존이 생긴다. 그래서 둘 다 여기를 바라보게 했다.

이 파일은 아무것도 import 하지 않는다. 의존 그래프의 맨 아래다.
"""

# 라벨 → (전진 배수, 회전 배수).
#
# 여기에는 "방향"만 두고 실제 속도는 파라미터로 곱한다.
# 방향은 로봇이 바뀌어도 그대로지만, 속도는 로봇마다 다르기 때문이다.
# 실제 곱셈은 GestureCommandPublisher 생성자에서 한다.
LABEL_MOTION = {
    'STOP':    (0.0, 0.0),
    'FORWARD': (1.0, 0.0),
    'LEFT':    (0.0, +1.0),   # angular.z 는 + 가 좌회전
    'RIGHT':   (0.0, -1.0),
}

# 발행이 허용된 라벨.
#
# LABEL_MOTION 에서 파생시켜 둘이 어긋날 수 없게 만든다.
# (원본 camera_node.py 는 두 곳에 손으로 적고 "키는 LABELS 와 일치해야 한다"는
#  주석으로 사람에게 관리를 맡기고 있었다)
LABELS = tuple(LABEL_MOTION)
