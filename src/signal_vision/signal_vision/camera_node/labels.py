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
#
# FORWARD (1.0, 0.0) 이 있었지만 뺐다. 그 라벨로 가는 유일한 입력이
# inference.LABEL_MAP 의 'come' 이었는데, 상류에서 come 을 수집 라벨에서 빼면서
# 모델이 낼 수 없는 라벨이 됐다. 정의만 남겨 두면 "전진 명령이 있다"고 읽히지만
# 실제로는 아무도 도달할 수 없는 죽은 항목이라, 그 상태를 코드에 남기지 않았다.
# 주행은 파견 라벨이 담당한다 — 아래 LABEL_DISPATCH 참고.
#
# 그래서 지금 남은 속도 라벨은 STOP 하나뿐이다. 그래도 이 딕셔너리를 유지하는
# 이유는 STOP 이 여전히 필요하기 때문이다: twist_mux 에서 gesture(50) 가
# auto(10) 를 이기므로, 파견 주행 중인 로봇을 사람이 세우는 수단이 이것이다.
# 전진 라벨이 다시 학습되면 여기에 한 줄 추가하고 LABEL_MAP 에 키를 이어 주면 된다.
LABEL_MOTION = {
    'STOP': (0.0, 0.0),
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
