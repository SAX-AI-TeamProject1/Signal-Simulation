import textwrap

from auto_drive.patrol.mission_follower import (load_dispatch_routes,
                                                station_indices)
import pytest


def _write_tracks(tmp_path, body):
    tracks_yaml = tmp_path / 'tracks.yaml'
    tracks_yaml.write_text(textwrap.dedent(body))
    return str(tracks_yaml)


def test_load_dispatch_routes_parses_and_finds_station(tmp_path):
    # 정상 케이스: 경로가 signal_point 에서 시작해 signal_point 로 끝나고,
    # 중간에 station(원판) 좌표가 끼어 있으면 그 인덱스가 잡혀야 한다.
    path = _write_tracks(tmp_path, """\
        signal_point: [0.0, 36.05]
        stations:
          zone_ne: [13.0, 13.0]
        routes:
          zone_ne:
            waypoints:
              - [0.0, 36.05]
              - [13.0, 37.8]
              - [13.0, 13.0]
              - [0.0, 14.3]
              - [0.0, 36.05]
        """)

    signal_point, stations, routes = load_dispatch_routes(path)

    assert signal_point == (0.0, 36.05)
    assert routes['zone_ne'][0] == routes['zone_ne'][-1] == (0.0, 36.05)
    assert station_indices(routes['zone_ne'], stations) == {2}


def test_load_dispatch_routes_rejects_route_not_returning_to_signal_point(tmp_path):
    # 파견은 수신호석 왕복이 계약이다 — 끝점이 다른 경로는 주행 중에 조용히
    # 엉뚱한 곳에 서는 대신 시작 시점에 실패해야 한다.
    path = _write_tracks(tmp_path, """\
        signal_point: [0.0, 36.05]
        routes:
          zone_ne:
            waypoints:
              - [0.0, 36.05]
              - [13.0, 13.0]
        """)

    with pytest.raises(RuntimeError):
        load_dispatch_routes(path)


def test_load_dispatch_routes_rejects_missing_routes(tmp_path):
    # routes 키 자체가 없으면(옛 포맷 tracks.yaml) 파견 노드는 뜰 이유가 없다.
    path = _write_tracks(tmp_path, """\
        signal_point: [0.0, 36.05]
        stations:
          zone_ne: [13.0, 13.0]
        """)

    with pytest.raises(RuntimeError):
        load_dispatch_routes(path)
