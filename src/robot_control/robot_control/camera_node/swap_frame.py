"""
캡처 스레드와 추론 워커 사이의 프레임 전달 통로.

책임 넷 중 하나가 아니라 배관(plumbing)이다. 다만 "낡은 프레임을 버린다"는
정책이 이 프로젝트의 안전과 직결돼서, 정책이 한 곳에 모여 있도록 파일을 나눴다.

원래는 frame_queue.py 에서 queue.Queue(maxsize=1) 로 만들었다. 그 파일은 삭제했고,
지금은 한 칸짜리 버퍼를 Condition 으로 직접 만든다. 큐를 버린 이유는 아래
LatestFrameBuffer 독스트링에 적어 뒀다. 옛 구현이 필요하면 git 이력에서 꺼낸다.
"""

import threading
import time


class LatestFrameBuffer:
    """
    캡처 스레드 → 워커 스레드로 프레임을 넘기는 한 칸짜리 버퍼.

    보관 정책은 옛 LatestFrameQueue 와 같다. 항상 최신 한 장만 들고 있고,
    새 프레임이 오면 낡은 것을 버린다. 로봇 제어에서 "30초 전 정지 신호"는
    "놓친 프레임"보다 훨씬 위험하기 때문이다. 다른 것은 구현 수단뿐이다.

    왜 queue.Queue(maxsize=1) 을 안 쓰는가:
        Queue 는 락 하나 위에 Condition 셋(not_empty / not_full / all_tasks_done)과
        join() 용 카운터를 얹은 물건인데, 여기서 필요한 건 not_empty 하나뿐이다.
        게다가 not_full 은 "가득 차면 생산자를 재우기"를 위한 것이라 이 노드가
        오히려 피해 다녀야 하는 기능이고(캡처 스레드는 절대 기다리면 안 된다),
        정작 필요한 "가득 차면 낡은 걸 버리고 덮어쓴다"는 Queue API 에 없다.

        그래서 옛 구현은 put_nowait 실패 → get_nowait → put_nowait 로 우회했다.
        그 get 과 put 사이에 워커가 프레임을 가져가면 get_nowait 이 Empty 를 내고,
        그 바람에 최신 프레임을 넣지도 못한 채 버리게 된다.
        (큐가 비어 있는데도 버린다 — 정책과 반대다)

        여기서는 폐기와 삽입이 대입문 하나라 그 틈이 아예 없다.

    Condition 이 곧 락이다. with self._cond 가 뮤텍스 획득이고, wait/notify 는
    거기에 "프레임 올 때까지 자는 기능"만 얹은 것이다. 락을 안 잡고 wait/notify 를
    부르면 RuntimeError 가 난다 — 선택이 아니라 API 계약이다.
    """

    def __init__(self):
        """빈 버퍼를 만든다."""
        self._cond = threading.Condition()  # lock 주입 안하면 내부적으로 재귀락
        self._slot = None

    def put_latest(self, frame):
        """
        프레임을 넣는다. 안에 있던 낡은 프레임은 덮어쓴다.

        반환:
            True  — 낡은 프레임을 한 장 버렸다(호출자가 유실로 집계)
            False — 버린 것 없이 그냥 들어갔다

        절대 블로킹되지 않는다. 캡처 스레드는 타이머 콜백 위에서 도는데
        여기서 기다리면 통계 타이머와 (앞으로 추가될) 구독 콜백까지 밀린다.
        락을 쥐는 구간이 대입 두 번뿐이라 실질적인 대기는 없다.
        """
        with self._cond:
            dropped = self._slot is not None
            self._slot = frame      # 폐기와 삽입이 한 연산 — 중간 틈이 없다
            self._cond.notify()     # 자고 있는 워커를 깨운다
        return dropped

    def get(self, timeout):
        """
        프레임을 꺼낸다. timeout 초 안에 없으면 None.

        타임아웃을 두는 이유: 프레임이 안 와도 워커가 주기적으로 깨어나
        종료 플래그를 확인해야 노드가 종료 시 매달리지 않는다.

        wait() 는 락을 놓는 것과 대기자 등록을 원자적으로 처리한다. 그래서
        "비었다고 판단한 시점"과 "대기 등록" 사이에 생산자가 끼어들어
        notify 가 허공에 날아가는 일(lost wakeup)이 없다.
        """
        with self._cond:
            # if 가 아니라 while: 깨어나서 락을 다시 잡았을 때 또 비어 있을 수
            # 있다(spurious wakeup). 소비자가 워커 하나뿐이라 실제로는 대개
            # 한 번에 통과하지만, 조건변수를 쓸 때의 기본형이다.
            #
            # 남은 시간을 다시 계산하는 이유: 헛되이 깨어날 때마다 timeout 을
            # 통째로 새로 주면 종료 시 워커가 예상보다 오래 매달릴 수 있다.
            deadline = time.monotonic() + timeout
            while self._slot is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                # timeout이거나 noti 오거나, 하면 wait 해제
                self._cond.wait(remaining)  # 임시 unlock
            frame, self._slot = self._slot, None
            return frame
