"""
캡처 스레드와 추론 워커 사이의 프레임 전달 통로.

책임 넷 중 하나가 아니라 배관(plumbing)이다. 다만 "낡은 프레임을 버린다"는
정책이 이 프로젝트의 안전과 직결돼서, 정책이 한 곳에 모여 있도록 파일을 나눴다.
"""

import queue


class LatestFrameQueue:
    """
    캡처 스레드 → 워커 스레드로 프레임을 넘기는 깊이 1 큐.

    깊이 1 = 항상 최신 프레임만. 왜 이래야 하는지:

        무한 큐:  캡처 30/초, 추론 20/초 → 초당 10장씩 쌓임
                  1분 뒤 600장 대기 = 30초 전 화면으로 판단(메모리도 터진다)   ✗
        깊이 1 :  낡은 프레임을 버리고 최신으로 교체 → 지연이 누적되지 않는다   ✓

    로봇 제어에서 "30초 전 정지 신호"는 "놓친 프레임"보다 훨씬 위험하다.
    버리는 것은 사고가 아니라 설계다. 다만 얼마나 버리는지는 세어 둔다(metrics).

    원본 camera_node.py 는 주석에 "깊이 1"이라 적어 놓고 코드는 maxsize=5 였다.
    깊이를 클래스 안에 가둬서 다시 어긋나지 않게 했다.
    """

    def __init__(self):
        """깊이 1 큐를 만든다."""
        self._queue = queue.Queue(maxsize=1)

    def put_latest(self, frame):
        """
        프레임을 넣는다. 자리가 없으면 안에 있던 낡은 프레임을 버리고 넣는다.

        반환:
            True  — 낡은 프레임을 한 장 버렸다(호출자가 유실로 집계)
            False — 버린 것 없이 그냥 들어갔다

        nowait 계열만 쓰는 이유: 그냥 put() 이면 자리가 날 때까지 블로킹되어
        타이머 콜백이 멈춘다. 캡처 스레드는 절대 기다리면 안 된다.
        """
        try:
            self._queue.put_nowait(frame)
            return False
        except queue.Full:
            pass

        # 여기 오는 경우: 워커가 아직 이전 프레임을 처리 중이다.
        # 생산자가 캡처 스레드 하나뿐이라 get 직후의 put 은 실패하지 않지만,
        # 워커가 그 찰나에 get 해 갈 수도 있으므로 예외는 그대로 흘려보낸다.
        try:
            self._queue.get_nowait()        # 낡은 프레임 폐기
            self._queue.put_nowait(frame)   # 최신 프레임 투입
        except (queue.Empty, queue.Full):
            pass
        return True

    def get(self, timeout):
        """
        프레임을 꺼낸다. timeout 초 안에 없으면 None.

        타임아웃을 두는 이유: 프레임이 안 와도 워커가 주기적으로 깨어나
        종료 플래그를 확인해야 노드가 종료 시 매달리지 않는다.
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None
