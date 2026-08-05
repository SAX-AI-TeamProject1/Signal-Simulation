#!/usr/bin/env python3
"""
stop_logger 가 남긴 CSV 를 정지 구간 그림과 요약으로 바꾼다.

stop_logger 는 '상태가 바뀐 순간'만 남긴다(auto_drive/safety/stop_logger.py).
구간으로 잇고 정지와 사유를 타임스탬프로 맞춰 보는 일은 전부 여기서 한다 —
오프라인이라 규칙이 틀리면 다시 주행하지 않고 이 파일만 고쳐 다시 돌리면 된다.

  python3 tools/plot_stops.py                       # 가장 최근 실행 기록을 창으로
  python3 tools/plot_stops.py run.csv --out run.png # 화면 없이 파일로
  python3 tools/plot_stops.py --selftest            # 구간 만드는 로직만 검사

라벨을 영어로 두는 이유는 matplotlib 기본 폰트에 한글이 없어서다(네모로 깨진다).
"""

import argparse
import csv
import pathlib

# 레인 순서. 정지를 맨 위에 두어 사유가 그 아래 깔리게 한다.
LANES = ['stopped', 'estop', 'gesture']


def to_lane(event):
    """이벤트 이름을 (레인, 켜짐인가) 로 바꾼다."""
    if event == 'stop':
        return 'stopped', True
    # move_start 가 현재 이름. move 는 옛 CSV 호환 — 이미 남은 기록은 다시
    # 주행해서 만들 수 없으므로 읽는 쪽이 두 이름을 다 받는다.
    if event in ('move_start', 'move'):
        return 'stopped', False
    if event.endswith('_on'):
        return event[:-3], True
    if event.endswith('_off'):
        return event[:-4], False
    raise ValueError(f'모르는 이벤트: {event!r}')


def build_intervals(events, end=None):
    """
    (시각, 이벤트) 목록을 (레인, 시작, 끝, 닫혔는가) 구간 목록으로 잇는다.

    닫히지 않은 채 로그가 끝난 구간은 로그의 끝(end)까지 이어 그린다 —
    노드가 죽거나 시뮬레이션이 끝난 것이지 구간이 없었던 게 아니다.
    end 는 tick(생존 신호)의 마지막 시각이다. 없이 부르면 마지막 전환
    시각으로 대신하는데, 그러면 마지막 전환 뒤 조용히 유지된 시간이 잘린다.
    """
    if not events:
        return []
    if end is None:
        end = events[-1][0]
    open_at = {}
    intervals = []
    for stamp, event in events:
        lane, is_open = to_lane(event)
        if is_open:
            # 같은 레인이 이미 열려 있으면 무시한다(중복 on 은 상태 변화가 아니다).
            open_at.setdefault(lane, stamp)
        elif lane in open_at:
            intervals.append((lane, open_at.pop(lane), stamp, True))
    for lane, stamp in open_at.items():
        intervals.append((lane, stamp, end, False))
    intervals.sort(key=lambda row: row[1])
    return intervals


def overlaps(first, second):
    """두 (시작, 끝) 구간이 겹치는가."""
    return first[0] < second[1] and second[0] < first[1]


def summarize(intervals):
    """정지 구간마다 그때 켜져 있던 사유와 반응 지연을 문자열 목록으로 만든다."""
    stops = [row for row in intervals if row[0] == 'stopped']
    reasons = [row for row in intervals if row[0] != 'stopped']
    lines = []
    for index, (_, start, end, closed) in enumerate(stops, 1):
        hit = [row for row in reasons if overlaps((start, end), (row[1], row[2]))]
        if hit:
            # 가장 먼저 켜진 사유가 이 정지를 일으켰다고 본다. 그 사유가 켜진
            # 시점과 실제로 선 시점의 차이가 반응 지연이다(음수면 이미 서 있었다).
            first = min(hit, key=lambda row: row[1])
            what = ', '.join(sorted({row[0] for row in hit}))
            lag = f'{start - first[1]:+.2f}s'
        else:
            what, lag = '(none)', '-'
        tail = '' if closed else ' [open]'
        lines.append(f'stop #{index:<3} t={start:8.2f}  dur={end - start:6.2f}s  '
                     f'reason={what:<16} lag={lag}{tail}')
    return lines


def totals(intervals):
    """레인별 (횟수, 총 시간)."""
    out = {}
    for lane, start, end, _ in intervals:
        count, total = out.get(lane, (0, 0.0))
        out[lane] = (count + 1, total + end - start)
    return out


def draw(intervals, out_path):
    """구간을 타임라인과 막대 두 장으로 그린다."""
    # matplotlib 을 함수 안에서 들여오는 이유: --selftest 는 화면도 이 라이브러리도
    # 필요 없고, 파일로 저장할 때는 창을 못 여는 환경일 수 있어 backend 를
    # pyplot 을 들여오기 전에 바꿔야 한다.
    import matplotlib
    if out_path:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    lanes = [lane for lane in LANES if any(row[0] == lane for row in intervals)]
    figure, (timeline, bars) = plt.subplots(2, 1, figsize=(12, 6),
                                            gridspec_kw={'height_ratios': [2, 1]})

    for index, lane in enumerate(lanes):
        spans = [(row[1], row[2] - row[1]) for row in intervals if row[0] == lane]
        timeline.broken_barh(spans, (index - 0.4, 0.8))
    timeline.set_yticks(range(len(lanes)))
    timeline.set_yticklabels(lanes)
    # matplotlib 은 0 번을 아래에 그린다. 뒤집어야 LANES 순서대로 위에서부터 쌓인다.
    timeline.invert_yaxis()
    timeline.set_xlabel('sim time (s)')
    timeline.set_title('stop intervals and active reasons')
    timeline.grid(axis='x', alpha=0.3)

    summary = totals(intervals)
    bars.bar(lanes, [summary[lane][1] for lane in lanes])
    for index, lane in enumerate(lanes):
        count, total = summary[lane]
        bars.text(index, total, f'{total:.1f}s / {count}x', ha='center', va='bottom')
    bars.set_ylabel('total seconds')
    bars.grid(axis='y', alpha=0.3)

    figure.tight_layout()
    if out_path:
        figure.savefig(out_path, dpi=120)
        print(f'saved: {out_path}')
    else:
        plt.show()


def load(path):
    """
    CSV 를 (전환 목록, 로그 끝 시각) 으로 읽는다. 전환은 시각순 정렬.

    tick 줄은 구간을 만들지 않는 생존 신호라 전환에서 걸러 내고, '로그가
    언제까지 살아 있었나'(끝 시각)를 정하는 데만 쓴다. tick 이 없는 옛
    CSV 는 끝 시각이 마지막 전환과 같아져 예전과 동일하게 동작한다.
    """
    with path.open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return [], None
    events = [(float(row['t_sim']), row['event'])
              for row in rows if row['stream'] != 'tick']
    events.sort(key=lambda row: row[0])
    end = max(float(row['t_sim']) for row in rows)
    return events, end


def selftest():
    """구간 잇기와 겹침 판정만 검사한다. ROS 도 matplotlib 도 필요 없다."""
    events = [(1.0, 'estop_on'), (1.5, 'stop'), (4.0, 'estop_off'),
              (4.5, 'move_start'), (9.0, 'gesture_on'), (9.2, 'stop')]
    intervals = build_intervals(events)

    assert ('estop', 1.0, 4.0, True) in intervals
    assert ('stopped', 1.5, 4.5, True) in intervals
    # 닫히지 않은 둘은 마지막 타임스탬프(9.2)까지 이어진다.
    assert ('gesture', 9.0, 9.2, False) in intervals
    assert ('stopped', 9.2, 9.2, False) in intervals

    # tick 이 로그의 끝을 정한다 — 열린 구간은 마지막 전환이 아니라 마지막
    # tick 까지 이어진다.
    assert build_intervals([(1.0, 'stop')], end=6.0) == \
        [('stopped', 1.0, 6.0, False)]

    assert overlaps((1.5, 4.5), (1.0, 4.0))
    assert not overlaps((1.5, 4.5), (4.5, 9.0))     # 맞닿기만 한 건 겹침이 아니다
    assert build_intervals([]) == []
    # 중복 on 은 상태 변화가 아니므로 처음 것만 남는다. 옛 이름 'move' 도
    # 그대로 읽혀야 한다(기존 CSV 호환).
    assert build_intervals([(1.0, 'stop'), (2.0, 'stop'), (3.0, 'move')]) == \
        [('stopped', 1.0, 3.0, True)]

    lines = summarize(intervals)
    assert 'reason=estop' in lines[0], lines[0]
    assert 'lag=+0.50s' in lines[0], lines[0]
    print('selftest ok')


def newest_log():
    """
    가장 최근 stop_log_*.csv 의 경로.

    bringup 은 <워크스페이스>/log/stops/ 에 남기므로 거기를 먼저 본다 — 이
    스크립트 자신의 위치(tools/) 기준이라 어느 디렉터리에서 실행해도 같은 곳을
    찾는다. ~/.ros 는 launch 없이 노드를 단독 실행했을 때의 fallback 위치라
    두 번째로 본다. 파일 이름이 시각으로 끝나므로 이름 정렬 = 시간 정렬이다.
    """
    ws_logs = pathlib.Path(__file__).resolve().parent.parent / 'log' / 'stops'
    for directory in (ws_logs, pathlib.Path.home() / '.ros'):
        found = sorted(directory.glob('stop_log_*.csv'))
        if found:
            return str(found[-1])
    return ''


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('csv', nargs='?', default=newest_log(),
                        help='stop_logger 가 남긴 CSV (기본: log/stops → ~/.ros '
                             '순서로 찾은 가장 최근 것)')
    parser.add_argument('--out', help='창을 열지 않고 이 경로에 PNG 로 저장')
    parser.add_argument('--selftest', action='store_true',
                        help='구간 만드는 로직만 검사하고 끝낸다')
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return
    if not args.csv:
        parser.error('log/stops 에도 ~/.ros 에도 stop_log_*.csv 가 없다 — '
                     '경로를 직접 넘길 것')

    events, end = load(pathlib.Path(args.csv))
    intervals = build_intervals(events, end)
    if not intervals:
        print('구간이 없다 — CSV 가 비었거나 전환이 한 번도 없었다')
        return

    for line in summarize(intervals):
        print(line)
    print()
    for lane, (count, total) in sorted(totals(intervals).items()):
        print(f'{lane:<10} {count:>4}x  {total:8.2f}s')
    draw(intervals, args.out)


if __name__ == '__main__':
    main()
