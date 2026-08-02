"""
수신호 라벨의 단일 출처(single source of truth).

왜 별도 파일인가:
    추론(inference.py)은 라벨을 "만들고", 제어 매핑(command_publisher.py)은
    라벨을 "소비한다". 두 모듈이 같은 정의를 봐야 하는데, 어느 한쪽에 두면
    다른 쪽이 그쪽을 import 해야 한다. 그러면 "추론이 제어를 안다" 같은
    엉뚱한 의존이 생긴다. 그래서 둘 다 여기를 바라보게 했다.

이 파일은 아무것도 import 하지 않는다. 의존 그래프의 맨 아래다.
"""

# 속도 라벨 → (전진 배수, 회전 배수).
#
# 여기에는 "방향"만 두고 실제 속도는 파라미터로 곱한다.
# 방향은 로봇이 바뀌어도 그대로지만, 속도는 로봇마다 다르기 때문이다.
# 실제 곱셈은 GestureCommandPublisher 생성자에서 한다.
LABEL_MOTION = {
    'STOP':    (0.0, 0.0),
    'FORWARD': (1.0, 0.0),
}

# 파견 라벨 → 목적지 존 이름(config/tracks.yaml 의 stations/routes 키).
#
# LEFT/RIGHT 는 원래 회전 Twist 였는데(angular.z ±), 신호가 끊긴 0.5초 뒤
# twist_mux 가 gesture 소스를 버리고 순찰이 도로 잡아 로봇을 트랙 방향으로
# 되돌리는 문제가 있었다. 그래서 회전이 아니라 "그 방향 존으로 다녀와라"는
# 파견 명령으로 바꿨다 — signal_dispatch(std_msgs/String)로 존 이름이 나가고,
# mission_follower(auto_drive)가 받아서 경로를 주행한다.
#
# 좌우의 기준: 신호수를 마주 보고 선 로봇 자신의 좌/우다. 로봇은 수신호석에서
# 북쪽(+y)의 신호수를 보므로 LEFT=서쪽(zone_nw). 신호수 기준으로 뒤집기로
# 정해지면 아래 두 값만 맞바꾸면 된다.
LABEL_DISPATCH = {
    'LEFT': 'zone_nw',
    'RIGHT': 'zone_ne',
}

# 발행이 허용된 라벨.
#
# 두 딕셔너리에서 파생시켜 어긋날 수 없게 만든다.
# (원본 camera_node.py 는 두 곳에 손으로 적고 "키는 LABELS 와 일치해야 한다"는
#  주석으로 사람에게 관리를 맡기고 있었다)
LABELS = tuple(LABEL_MOTION) + tuple(LABEL_DISPATCH)
