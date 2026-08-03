"""
책임 4 — 라벨을 로봇 속도 명령으로 바꿔 발행.

config/twist_mux.yaml 은 이 변환을 별도 command_node 가 맡는 것으로 적고 있다.
지금 같은 프로세스에 두는 이유는 변환이 딕셔너리 조회 한 번이라 노드를 하나 더
띄울 만큼의 일이 아니어서다. 다만 파일로는 분리해 두어, 나중에 노드로 떼어낼 때
이 파일을 그대로 옮기면 되게 했다.
"""

from geometry_msgs.msg import Twist
from signal_vision.camera_node.labels import (LABEL_DISPATCH, LABEL_MOTION,
                                              LABELS)
from signal_vision.camera_node.shutdown import is_shutting_down
from std_msgs.msg import String


class GestureCommandPublisher:
    """
    라벨 문자열을 받아 세 토픽으로 내보낸다.

        gesture (std_msgs/String)          — 관측·기록용.
            rosbag 에 남겨야 "언제 무슨 수신호였나"를 알 수 있고,
            "인식→반응 지연시간" 측정도 이 타임스탬프가 있어야 한다.

        cmd_vel_gesture (geometry_msgs/Twist) — 속도 라벨(지금은 STOP 하나) 제어용.
            twist_mux 의 gesture 입력(priority 50)으로 들어간다.
            리맵은 없다. bringup 이 이 노드를 로봇 네임스페이스 안에서 띄우므로
            발행 토픽이 자동으로 <ns>/cmd_vel_gesture 가 되고, 같은 네임스페이스의
            twist_mux 가 그대로 구독한다. 예전에는 이 노드가 로봇 밖에 있어서
            '/robot1/cmd_vel_gesture' 로 하드코딩 리맵을 걸었는데, robot1 이
            빠지면서 아무도 듣지 않는 토픽을 가리키게 된 적이 있다.

        signal_dispatch (std_msgs/String)  — 파견 라벨(LEFT/RIGHT) 제어용.
            twist_mux 를 거치지 않는다 — 속도가 아니라 "존 이름"이고, 받는 쪽은
            같은 네임스페이스의 mission_follower(auto_drive)다. 회전 Twist 를
            버리고 파견으로 바꾼 이유는 labels.py 의 LABEL_DISPATCH 주석 참고.

    QoS 는 셋 다 depth 10 + 기본 RELIABLE. 영상과 달리 라벨은 유실되면 안 된다
    (특히 STOP). 그래서 BEST_EFFORT 를 쓰지 않는다.
    """

    def __init__(self, node, stop_event, linear_speed, angular_speed,
                 gesture_topic='gesture', cmd_vel_topic='cmd_vel_gesture',
                 dispatch_topic='signal_dispatch', zone_gate=None):
        """
        퍼블리셔 2개를 만들고, 라벨별 Twist 를 미리 만들어 캐싱한다.

        인자:
            linear_speed: 전진 속도 (m/s) — 지금은 전진 라벨이 없어 예약값이다.
                유일한 입력이던 come 이 학습 라벨에서 빠져 FORWARD 를 지웠다
                (배경은 labels.py 의 LABEL_MOTION 주석).
            angular_speed: 회전 속도 (rad/s) — 지금은 회전 라벨이 없어 예약값이다.
                LEFT/RIGHT 가 파견 라벨이 되면서 쓰는 곳이 없어졌다.
            둘 다 파라미터와 생성자 시그니처는 유지한다. 라벨이 다시 생기면
            LABEL_MOTION 에 한 줄 넣는 것만으로 그대로 쓰인다.
            zone_gate: SignalZoneGate 또는 None. 있으면 allows() 가 참일 때만
                cmd_vel_gesture/signal_dispatch 를 발행한다. gesture(String) 는
                게이트와 무관하게 항상 나간다 — 관측·기록용이라 "존 밖에서 무슨
                수신호가 잡혔나"도 rosbag 에 남아야 하기 때문이다.

        Twist 를 미리 만드는 이유: 라벨이 몇 개 안 되어 매 프레임 새로 만들 이유가 없다.
        캐시된 객체를 재발행해도 되는 것은 publish() 가 내부에서 직렬화하기 때문이다
        (발행 후 그 객체를 고치지만 않으면 된다 → 이 클래스 밖으로 내보내지 않는다).
        """
        if linear_speed < 0.0 or angular_speed < 0.0:
            raise ValueError(
                f'속도는 음수일 수 없습니다 (linear={linear_speed}, angular={angular_speed}). '
                '방향은 LABEL_MOTION 의 부호가 정합니다.')

        self._node = node
        self._stop_event = stop_event
        self._zone_gate = zone_gate
        self._last_label = None     # 워커 스레드에서만 읽고 쓴다 → 락 불필요

        self._pub_gesture = node.create_publisher(String, gesture_topic, 10)
        self._pub_cmd_vel = node.create_publisher(Twist, cmd_vel_topic, 10)
        self._pub_dispatch = node.create_publisher(String, dispatch_topic, 10)

        self._label_to_twist = {}
        for label, (lin_mul, ang_mul) in LABEL_MOTION.items():
            twist = Twist()
            twist.linear.x = linear_speed * lin_mul
            twist.angular.z = angular_speed * ang_mul
            self._label_to_twist[label] = twist

    def publish(self, label):
        """
        라벨 하나를 발행한다. 워커 스레드에서 호출된다.

        (rclpy 퍼블리셔는 스레드 안전하므로 워커에서 바로 발행해도 된다)

        인자:
            label: LABELS 중 하나. None 이거나 목록에 없으면 발행하지 않는다.

        반환:
            True  — 발행했거나, 발행할 것이 없어 건너뛰었다
            False — 종료 중이다(호출자는 워커 루프를 빠져나와야 한다)

        ┌─ 라벨이 끊겨도 유지(hold/latch)하지 않는다 ──────────────────────┐
        │ twist_mux 의 gesture timeout 은 0.5초다. label 이 None 인 동안은  │
        │ 여기서 아무것도 발행하지 않으므로, 인식이 0.5초 넘게 끊기면        │
        │ gesture 소스가 버려진다. 마지막 라벨을 붙잡아 둘 수도 있지만       │
        │ 그러지 않기로 했다.                                                │
        │                                                                   │
        │ 남은 속도 라벨은 STOP 하나뿐이고, 그건 붙잡을 이유가 없는 쪽이다.  │
        │ 신호수가 손을 들고 있는 동안은 매 프레임 다시 나가고, 손을 내리면  │
        │ 그건 "이제 가도 된다"는 뜻이다. 유지하면 사람이 이미 낸 해제       │
        │ 신호를 로봇이 무시하게 된다.                                       │
        │                                                                   │
        │ 놓쳐서 위험해지는 경우도 없다. 수신호가 로봇을 움직이는 라벨이     │
        │ 없고, 파견 주행 중에는 zone_gate 가 닫혀 있어(in_zone and facing   │
        │ — signal_zone.py) STOP 이 애초에 나가지 않는다. 대기 중이면        │
        │ mission_follower 도 발행을 멈추므로 아무도 움직이지 않는다.        │
        │ 주행 중 안전은 estop_node(우선순위 100)의 몫이다.                  │
        └───────────────────────────────────────────────────────────────────┘
        """
        if label is None:
            return True
        if label not in LABELS:
            # fatal 이 아니라 warn: 무시하고 계속 도는 상황이라 치명적이지 않다.
            # 모델이 오타 난 라벨을 뱉는 흔한 실수라서 눈에는 띄어야 한다.
            self._node.get_logger().warn(
                f'정의되지 않은 라벨이라 무시합니다: {label!r} (허용: {LABELS})')
            return True

        # infer() 가 도는 사이에 종료가 시작됐을 수 있다 → 발행 직전에 확인.
        if is_shutting_down(self._stop_event):
            return False

        try:
            # 라벨이 바뀔 때만 로그. 매 프레임 찍으면 초당 20줄이 쏟아진다.
            if label != self._last_label:
                self._node.get_logger().info(f'수신호 인식: {label}')
                self._last_label = label
            self._pub_gesture.publish(String(data=label))
            # 게이트가 닫혀 있으면 명령만 삼킨다. 발행을 그냥 안 하면 되는 이유는
            # estop 의 해제와 같다 — twist_mux 가 timeout(0.5초)으로 gesture
            # 소스를 버리므로 별도의 "무효화" 신호가 필요 없다. 파견도 같은
            # 게이트를 지난다: 수신호석 밖에서 웹캠에 잡힌 LEFT 로 임무가 시작되면
            # 안 되는 건 회전 명령 시절과 똑같다.
            if self._zone_gate is not None and not self._zone_gate.allows():
                return True
            if label in LABEL_DISPATCH:
                # 파견 라벨은 속도가 아니라 존 이름으로 나간다. 라벨이 잡히는 동안
                # 매 프레임 다시 나가지만, mission_follower 가 주행 중 수신을
                # 무시하므로 중복 발행은 해가 없다(대기 중 첫 수신만 유효).
                self._pub_dispatch.publish(String(data=LABEL_DISPATCH[label]))
            else:
                self._pub_cmd_vel.publish(self._label_to_twist[label])
        except RuntimeError:
            # 위 검사와 발행 사이에 종료가 끼어든 경우.
            self._stop_event.set()
            return False
        return True
