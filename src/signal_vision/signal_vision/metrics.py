"""
추론 파이프라인의 처리량·지연 통계를 모으는 유틸리티.

노드 파일에서 분리한 이유: inline 방식(camera_node 안에서 추론)과 split 방식(별도 AI 노드)을
비교하려면 측정 방법과 출력 형식이 같아야 한다. 양쪽이 이 클래스를 쓰면 그게 보장된다.

출력은 drain_report() 호출 사이의 **구간값**이다. stats_period 를 1초로 두면 그대로 Hz 가 된다.

스레드 규칙:
- 지연 표본 리스트만 락으로 보호한다. 워커가 append 하는 사이 통계 타이머가 정렬하면
  "ValueError: list modified during sort" 가 날 수 있기 때문이다.
- 카운터 3개는 락이 없다. 각각 한 스레드에서만 증가하기 때문이다.
  (record_capture / record_drop → 캡처 스레드,  record_inference → 추론 워커)
  이 전제가 깨지면 카운트가 유실될 수 있으니, 호출 스레드를 바꿀 때 이 주석을 확인할 것.
"""

import threading


class InferenceMetrics:
    """캡처/추론/유실 횟수와 추론 소요시간을 모아 구간 통계로 낸다."""

    def __init__(self):
        """빈 통계로 시작한다."""
        self._lock = threading.Lock()
        self._latencies_ms = []

        # 누적 카운터. 각각 한 스레드만 증가시킨다.
        self._captured = 0
        self._inferred = 0
        self._dropped = 0

        # 직전 보고 시점의 누적값. 구간값을 차이로 구하려고 둔다.
        # 읽는 쪽(drain_report)만 건드리므로 경쟁 대상이 아니다.
        self._prev_captured = 0
        self._prev_inferred = 0
        self._prev_dropped = 0

    def record_capture(self):
        """프레임을 한 장 캡처했다. 캡처 스레드에서만 호출한다."""
        self._captured += 1

    def record_drop(self):
        """큐가 차서 프레임을 한 장 버렸다. 캡처 스레드에서만 호출한다."""
        self._dropped += 1

    def record_inference(self, elapsed_ms):
        """추론 한 번이 끝났다(elapsed_ms = 소요시간). 추론 워커에서만 호출한다."""
        self._inferred += 1
        with self._lock:
            self._latencies_ms.append(elapsed_ms)

    def drain_report(self):
        """
        마지막 호출 이후 구간의 통계를 한 줄 문자열로 만든다.

        카운터를 0 으로 리셋하지 않고 직전값과의 차이로 구간값을 구한다.
        리셋하면 읽는 쪽(실행기 스레드)이 _inferred 에 쓰게 되는데, 그 값은
        추론 워커도 증가시키므로 두 스레드가 같은 값을 쓰게 되어 증가분이 유실된다.
        차이 방식은 원본 카운터를 단일 작성자로 유지한다.

        정렬을 락 밖에서 하는 이유: 스왑으로 리스트를 떼어낸 순간
        samples 는 이 호출만의 것이 되어 다른 스레드가 건드리지 않는다.
        락 안에서 정렬하면 그동안 워커의 record_inference 가 막힌다.

        thread A write
        thread B read
        에 대해서, B thread는 코어 캐시 값이 아니라, 실제 값을 가져온다(GIL이라는 글로벌락으로 가능하다고 하네요{잘 모름;})
        """
        with self._lock:
            samples, self._latencies_ms = self._latencies_ms, []

        # 세 값을 한 번만 읽어 스냅샷으로 쓴다(읽는 사이 값이 늘어도 다음 구간에 잡힌다).
        captured, inferred, dropped = self._captured, self._inferred, self._dropped
        d_captured = captured - self._prev_captured
        d_inferred = inferred - self._prev_inferred
        d_dropped = dropped - self._prev_dropped
        self._prev_captured = captured
        self._prev_inferred = inferred
        self._prev_dropped = dropped

        if samples:
            samples.sort()
            latency = (f'추론 median={samples[len(samples) // 2]:.1f}ms '
                       f'max={samples[-1]:.1f}ms')
        else:
            latency = '추론 표본 없음'
        return f'캡처={d_captured} 추론={d_inferred} 유실={d_dropped} | {latency}'
