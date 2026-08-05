"""
로봇이 언제 섰고, 그때 어떤 정지 사유가 켜져 있었는지를 CSV 에 남긴다.

주행에는 관여하지 않는다 — 이미 발행 중인 토픽만 구독하는 관측 노드다.
그래서 estop_node 도 camera_node 도 고치지 않는다.

기록 형식은 '구간'이 아니라 '상태가 바뀐 순간'이다. 한 줄이 곧 한 전환이고,
구간으로 잇는 일은 tools/plot_stops.py 가 나중에 한다. 구간을 여기서 만들지
않는 이유가 둘 있다:
  - 정지와 사유는 같은 것이 아니다. 사유가 켜져 있던 시간을 정지 시간으로
    적으면, 이미 서 있는 로봇 앞으로 사람이 지나갈 때 정지가 중복으로 잡히고,
    사유가 켜지고 실제로 서기까지의 지연은 아예 기록에 안 남는다. 둘을 따로
    남기고 타임스탬프로 맞춰 보면 그 지연이 그대로 보인다.
  - 판정 규칙이 틀렸을 때 다시 주행하지 않아도 된다. CSV 는 그대로 두고
    그리는 스크립트만 고쳐 다시 돌리면 된다.

두 스트림:
  motion — 실제로 섰는지. odom 의 속도가 임계값 아래면 stop, 위면 move_start.
      명령(cmd_vel)이 아니라 실측을 쓰는 이유는 '얼마나 정지했는지'가 물리
      시간이기 때문이다. 명령만 보면 명령은 갔는데 못 움직인 경우를 놓친다.
  reason — 정지를 만들 수 있는 소스가 켜져 있는지. estop(라이다·스캔 끊김)과
      cmd_vel_gesture(수신호 STOP) 둘이다. 서로 독립이라 동시에 켜져 있어도
      각자 찍힌다 — 우선순위로 하나만 남기면 '라이다가 안 세웠으면 수신호가
      세웠을' 상황이 기록에서 사라진다.

tick 스트림은 '전환만 남긴다'의 유일한 예외다 — 전환이 없어도 0.5초마다
생존 신호 한 줄을 남긴다. 이게 없으면 마지막 전환 뒤 조용히 유지된 구간은
길이가 잘려 나오고(그리는 쪽이 마지막 타임스탬프를 로그의 끝으로 보므로),
'상태가 그대로였던 것'과 '이 노드가 죽은 것'을 로그만 보고 구분할 수 없다.

라이다 정지와 스캔 끊김은 나누지 않는다. 그림에서 둘 다 'estop 이 로봇을
붙잡고 있던 구간'으로 똑같이 그려지므로 구분이 결과를 바꾸지 않는다. 나중에
'스캔이 끊겨서 선 적이 몇 번인가'를 실제로 묻게 되면, estop_node 의
_last_reason 을 String 으로 발행하고(그 값은 거기 이미 있다) 여기서 그 토픽을
받으면 된다 — 스캔을 다시 구독해 판정을 되풀이할 일이 아니다.

시각은 두 벌을 남긴다. t_sim 은 use_sim_time 으로 받는 /clock 기준이라 gz 의
일시정지·배속과 함께 움직이고, t_wall 은 사람이 '몇 시에 있었던 일'인지
찾을 때 쓴다. 노드가 gz 보다 먼저 뜨면 첫 /clock 이 오기 전까지 t_sim 이 0
근처로 찍히는데, 그 구간은 t_wall 로 갈라 볼 수 있다.

사유 문자열이 한글이 아닌 이유: tools/plot_stops.py 의 matplotlib 기본 폰트에
한글이 없어 축 라벨이 네모로 깨진다.
"""

import csv
import datetime
import math
import pathlib

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

# CSV 한 줄 = 상태 전환 한 번. 네 열의 의미:
#   t_sim  — 시뮬레이션 시각(초, /clock 기준). gz 의 일시정지·배속과 함께
#            움직이므로 구간 길이 계산은 이 열로 한다.
#   t_wall — 벽시계 시각(ISO 8601, ms 까지). '몇 시에 있었던 일인지' 사람이
#            찾을 때와, /clock 이 오기 전 t_sim 이 0 근처인 구간을 가를 때 쓴다.
#   stream — 이 줄이 속한 계열. motion(odom 실측으로 실제로 섰는가),
#            reason(정지를 만들 수 있는 소스가 켜져 있는가),
#            tick(전환 없는 생존 신호 — 구간이 아니라 로그의 끝을 알린다).
#            전부 한 파일에 섞어 두고 이 열로 구분한다 — 파일이 하나여야
#            시간축 정렬이 공짜다.
#   event  — 줄의 내용. motion 이면 stop/move_start,
#            reason 이면 estop_on/estop_off/gesture_on/gesture_off,
#            tick 이면 alive.
CSV_HEADER = ['t_sim', 't_wall', 'stream', 'event']


class StopLogger(Node):
    """정지(odom)와 정지 사유(estop·수신호)의 전환 순간을 CSV 에 남긴다."""

    def __init__(self):
        super().__init__('stop_logger')

        # 빈 값이면 ~/.ros/stop_log_<시각>.csv — launch 없이 단독 실행할 때의
        # fallback. bringup 은 워크스페이스의 log/stops/ 절대 경로를 만들어 이
        # 파라미터로 넘긴다. 노드가 상대 경로를 스스로 풀지 않는 이유: 상대
        # 경로는 프로세스 CWD(명령을 친 위치) 기준이라 실행 위치마다 기록이
        # 흩어진다 — 앵커를 아는 건 launch 쪽이다.
        # 파일 이름에 시각을 넣어 실행마다 새 파일이 되게 한다 — 한 파일에 이어
        # 붙이면 t_sim 이 매 실행 0 부터 다시 시작해 구간이 서로 엉킨다.
        self.declare_parameter('csv_path', '')

        # 정지/주행 판정 임계값을 다르게 둔다(히스테리시스). 같은 값이면 로봇이
        # 그 속도 근처에서 흔들릴 때 stop/move 가 초당 수십 줄씩 쏟아진다.
        # 0.03 은 정지 중 수치 잡음보다 크고, 0.08 은 순항(3.33 m/s)의 2.4% 라
        # '움직이기 시작했다'를 놓치지 않는다.
        self.declare_parameter('stop_speed', 0.03)
        self.declare_parameter('move_speed', 0.08)

        # 수신호 정지의 끝은 메시지가 아니라 '침묵'이다. 그 침묵의 길이는
        # config/twist_mux.yaml 의 gesture timeout 과 같은 값을 쓴다 — 실제로
        # 로봇이 붙잡혀 있던 시간과 어긋나지 않게 하려는 것. 저쪽을 바꾸면
        # 여기도 같이 바꿔야 한다.
        self.declare_parameter('gesture_timeout_sec', 0.5)

        # 생존 신호 주기. 열린 구간의 끝이 최대 이만큼 짧게 잡히므로, 수신호
        # timeout(0.5초)과 같은 해상도로 맞춘다. 0.5초 = 시간당 7200줄,
        # 수백 KB — 부담 없는 크기다. 0 이하면 끈다.
        self.declare_parameter('tick_sec', 0.5)

        path = self.get_parameter('csv_path').value
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = (pathlib.Path(path) if path
                          else pathlib.Path.home() / '.ros' / f'stop_log_{stamp}.csv')
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        # 헤더 판정은 열기 전에 해야 한다 — 'a' 로 여는 순간 파일이 생겨 버린다.
        # (기본 경로는 실행마다 새 이름이라 항상 새 파일이지만, csv_path 를 직접
        #  넘겨 이어 쓰는 경우에 헤더가 중간에 또 박히면 안 된다)
        is_new = not self._csv_path.exists()
        self._file = self._csv_path.open('a', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        if is_new:
            self._write_row(CSV_HEADER)

        # None = 아직 판정 전. 첫 메시지에서 정해지고 그때 한 줄이 남는다.
        self._stopped = None
        self._estop_on = None
        self._gesture_on = False
        self._last_gesture_sec = 0.0

        # odom 은 ground_truth_tf 가 쓰는 것과 같은 QoS 로 받는다(같은 발행자).
        self.create_subscription(Odometry, 'odom', self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(Twist, 'cmd_vel_gesture', self._on_gesture, 10)

        # 수신호 구간을 닫기 위한 타이머. timeout 보다 촘촘해야 닫히는 시점이
        # timeout 을 한 주기 이상 넘기지 않는다.
        self.create_timer(0.1, self._on_timer)

        # use_sim_time 이면 이 타이머도 sim 시계를 따르므로, 첫 /clock 이 오기
        # 전에는 tick 이 찍히지 않는다 — t_sim 0 짜리 줄이 쌓이는 일은 없다.
        tick_sec = self.get_parameter('tick_sec').value
        if tick_sec > 0.0:
            self.create_timer(tick_sec, self._on_tick)

        self.get_logger().info(f'정지 기록: {self._csv_path}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        # 각속도도 함께 본다 — 제자리 회전은 이동이 아니지만 정지도 아니다.
        # 둘 중 큰 쪽으로 재면 '둘 다 느리다'와 '하나라도 빠르다'가 그대로 나온다.
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        fastest = max(speed, abs(msg.twist.twist.angular.z))

        if fastest < self.get_parameter('stop_speed').value:
            self._set_motion(True)
        elif fastest > self.get_parameter('move_speed').value:
            self._set_motion(False)
        # 두 임계값 사이는 불감대라 직전 상태를 그대로 둔다. 임계값이 하나면
        # 그 값 근처에서 흔들릴 때마다 stop/move 가 번갈아 쏟아진다.

    def _set_motion(self, stopped):
        # odom 은 초당 수십 번 들어오지만 남기는 건 뒤집힐 때 한 줄뿐이다.
        # 판정은 부르는 쪽이 하고, '바뀌었을 때만'은 상태를 가진 여기가 맡는다.
        # 처음 한 번은 _stopped 가 None 이라 무엇이 오든 전이로 잡힌다.
        if stopped == self._stopped:
            return
        self._stopped = stopped
        self._write_event('motion', 'stop' if stopped else 'move_start')

    def _on_estop(self, msg):
        # estop_node 는 True/False 를 20Hz 로 계속 쏜다. 바뀔 때만 한 줄 남긴다.
        if msg.data == self._estop_on:
            return
        self._estop_on = msg.data
        self._write_event('reason', 'estop_on' if msg.data else 'estop_off')

    def _on_gesture(self, msg):
        # 지금 수신호 라벨은 STOP 하나뿐이라 항상 0 이지만, 전진 라벨이 다시
        # 생기면 그건 정지가 아니다. 값으로 판정해 둔다.
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self._last_gesture_sec = self._now()
            if not self._gesture_on:
                self._gesture_on = True
                self._write_event('reason', 'gesture_on')
        elif self._gesture_on:
            self._gesture_on = False
            self._write_event('reason', 'gesture_off')

    def _on_timer(self):
        # 수신호 구간을 닫는 유일한 경로가 이 타이머다. STOP 의 끝은 '침묵'이라
        # _on_gesture 는 그 순간을 볼 수 없고(콜백은 메시지가 와야 불린다),
        # 거기의 elif 는 0 이 아닌 명령이 왔을 때만 타는데 지금 라벨로는 안 온다.
        # timeout 이 곧 정상적인 off 라서 이벤트도 gesture_off 하나로 남긴다 —
        # 이름을 나누면 그리는 쪽이 같은 것을 두 번 배워야 한다.
        if not self._gesture_on:
            return
        if self._now() - self._last_gesture_sec > \
                self.get_parameter('gesture_timeout_sec').value:
            self._gesture_on = False
            self._write_event('reason', 'gesture_off')

    def _on_tick(self):
        # 내용 없는 생존 신호. 그리는 쪽은 이 줄로 구간을 만들지 않고
        # '로그가 여기까지 살아 있었다'로만 쓴다(plot_stops.load 가 걸러 읽는다).
        self._write_event('tick', 'alive')

    def _write_event(self, stream, event):
        wall = datetime.datetime.now().isoformat(timespec='milliseconds')
        self._write_row([f'{self._now():.3f}', wall, stream, event])

    def _write_row(self, row):
        self._writer.writerow(row)
        # 파일에 남기는 건 close 가 아니라 flush 다. 전환 + 0.5초 tick 수준의
        # 빈도라 한 줄마다 흘려보내도 비용이 없고, 이러면 노드가 죽어도
        # 그때까지의 줄은 파일에 있다. fsync 까지는 안 한다 — 프로세스가 죽는
        # 건 flush 로 막히고, 그 위(전원 차단)는 이 기록으로 지킬 값이 아니다.
        self._file.flush()

    def destroy_node(self):
        """파일 핸들을 닫고 노드를 내린다."""
        self._file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StopLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 닫히지 않은 구간을 여기서 마무리할 필요가 없다. 전환만 남기는 형식이라
        # '마지막 줄 이후로 계속 그 상태였다'가 이미 기록이고, 그리는 쪽이
        # 마지막 타임스탬프까지 이어 그린다.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
