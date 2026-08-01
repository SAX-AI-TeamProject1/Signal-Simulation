import math
import textwrap

from auto_drive.patrol.waypoint_follower import (classify_waypoints,
                                                 load_track_points,
                                                 lookahead_point,
                                                 WAYPOINTS)


def test_classify_waypoints_matches_config_tracks_yaml(tmp_path):
    # WAYPOINTS는 [출발, ...허브 회차루프 경유점들..., 도착] 순서다 — 첫 점과 마지막
    # 점만 signal_point/station과 겹치고, 중간(회차루프 등)은 전부 corner여야 한다.
    start_x, start_y = WAYPOINTS[0]
    end_x, end_y = WAYPOINTS[-1]
    tracks_yaml = tmp_path / 'tracks.yaml'
    tracks_yaml.write_text(textwrap.dedent(f"""\
        signal_point: [{start_x}, {start_y}]
        stations:
          south_end: [{end_x}, {end_y}]
        tracks:
          ne_entry_patrol:
            loop: true
            waypoints:
              - [{start_x}, {start_y}, -1.5708]
              - [{end_x}, {end_y}, -1.5708]
        """))

    signal_point, stations = load_track_points(str(tracks_yaml))
    types = classify_waypoints(WAYPOINTS, signal_point, stations)

    assert types == (['signal_point'] + ['corner'] * (len(WAYPOINTS) - 2) + ['station'])


def test_classify_waypoints_defaults_to_corner_without_config():
    types = classify_waypoints(WAYPOINTS, None, [])
    assert types == ['corner'] * len(WAYPOINTS)


def test_lookahead_point_advances_along_segment_when_on_track():
    aim = lookahead_point((0.0, 0.0), (10.0, 0.0), (3.0, 0.0), lookahead_dist=2.0)
    assert math.isclose(aim[0], 5.0)
    assert math.isclose(aim[1], 0.0)


def test_lookahead_point_pulls_back_toward_line_when_off_track():
    # 로봇이 트랙 선(y=0)에서 y=1.5만큼 벗어나 있어도, 조준점은 트랙 선 위에 있어야
    # 되돌아오도록 유도한다.
    aim = lookahead_point((0.0, 0.0), (10.0, 0.0), (3.0, 1.5), lookahead_dist=2.0)
    assert math.isclose(aim[0], 5.0)
    assert math.isclose(aim[1], 0.0)


def test_lookahead_point_clamped_to_segment_near_target():
    aim = lookahead_point((0.0, 0.0), (10.0, 0.0), (9.5, 0.0), lookahead_dist=2.0)
    assert math.isclose(aim[0], 10.0)
    assert math.isclose(aim[1], 0.0)
