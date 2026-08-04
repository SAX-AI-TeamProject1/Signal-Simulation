#!/usr/bin/env python3
# Last updated: 2026-07-20
"""Generate visual-only floor markers for tracks and points from tracks.yaml.

Emits one static model containing:
  - a painted line strip along every track (per-track color),
  - a red disc at the signal point,
  - a green disc per station point.

The visuals have no <collision>, so they change nothing physically — vehicles
and LiDAR rays are unaffected; they only make the layout visible on the floor.

Usage:
    # print the marker model SDF (--tracks defaults to navi_factory's tracks.yaml,
    # which sits next to the world SDF it describes)
    python3 tools/tracks_to_markers.py

    # write/replace it inside a world file, between the markers
    # <!-- BEGIN GENERATED TRACK MARKERS --> ... <!-- END GENERATED TRACK MARKERS -->
    python3 tools/tracks_to_markers.py \
        --inject worlds/navi_factory/world/navi_factory/navi_factory.sdf
"""

import argparse
import math
import pathlib
import re

import yaml

BEGIN_MARK = "<!-- BEGIN GENERATED TRACK MARKERS -->"
END_MARK = "<!-- END GENERATED TRACK MARKERS -->"

TRACK_COLORS = [
    (1.0, 0.8, 0.0),  # yellow
    (0.0, 0.6, 1.0),  # blue
    (0.7, 0.3, 1.0),  # purple
    (1.0, 0.5, 0.0),  # orange
]
SIGNAL_COLOR = (1.0, 0.1, 0.1)
STATION_COLOR = (0.1, 0.8, 0.2)

Z = 0.015  # just above the floor to avoid z-fighting


def material(rgb):
    r, g, b = rgb
    return (
        "        <material>\n"
        f"          <ambient>{r} {g} {b} 1</ambient>\n"
        f"          <diffuse>{r} {g} {b} 1</diffuse>\n"
        "        </material>"
    )


def line_visual(name, p0, p1, rgb, width):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
    yaw = math.atan2(dy, dx)
    return (
        f'      <visual name="{name}">\n'
        f"        <pose>{round(mx, 3)} {round(my, 3)} {Z} 0 0 {round(yaw, 4)}</pose>\n"
        "        <geometry>\n"
        f"          <box><size>{round(length, 3)} {width} 0.002</size></box>\n"
        "        </geometry>\n"
        f"{material(rgb)}\n"
        "      </visual>"
    )


def disc_visual(name, x, y, rgb, radius):
    return (
        f'      <visual name="{name}">\n'
        f"        <pose>{x} {y} {Z} 0 0 0</pose>\n"
        "        <geometry>\n"
        f"          <cylinder><radius>{radius}</radius><length>0.002</length></cylinder>\n"
        "        </geometry>\n"
        f"{material(rgb)}\n"
        "      </visual>"
    )


def generate(tracks_path, width):
    with open(tracks_path) as f:
        data = yaml.safe_load(f)

    visuals = []
    for i, (name, track) in enumerate((data.get("tracks") or {}).items()):
        points = track.get("waypoints", [])
        if len(points) < 2:
            continue
        rgb = TRACK_COLORS[i % len(TRACK_COLORS)]
        segments = list(zip(points, points[1:]))
        if track.get("loop"):
            segments.append((points[-1], points[0]))
        for j, (p0, p1) in enumerate(segments):
            visual = line_visual(f"{name}_seg_{j}", p0, p1, rgb, width)
            if visual:
                visuals.append(visual)

    signal_point = data.get("signal_point")
    if signal_point:
        visuals.append(
            disc_visual("signal_point", signal_point[0], signal_point[1],
                        SIGNAL_COLOR, 0.4)
        )
    for name, point in (data.get("stations") or {}).items():
        visuals.append(disc_visual(name, point[0], point[1], STATION_COLOR, 0.3))

    if not visuals:
        return ""
    return (
        '<model name="track_markers">\n'
        "  <static>true</static>\n"
        '  <link name="markers">\n'
        + "\n".join(visuals)
        + "\n  </link>\n</model>"
    )


def inject(world_path, snippet):
    world_path = pathlib.Path(world_path)
    text = world_path.read_text()
    indent = "    "
    block = (
        f"{indent}{BEGIN_MARK}\n"
        + "\n".join(indent + line for line in snippet.splitlines())
        + f"\n{indent}{END_MARK}"
    )
    if BEGIN_MARK in text:
        pattern = re.escape(f"{indent}{BEGIN_MARK}") + r".*?" + re.escape(END_MARK)
        text = re.sub(pattern, block.lstrip(), text, flags=re.DOTALL)
    else:
        text = text.replace("</world>", f"{block}\n  </world>", 1)
    world_path.write_text(text)
    print(f"injected track markers into {world_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tracks",
        default="worlds/navi_factory/world/navi_factory/tracks.yaml")
    parser.add_argument("--width", type=float, default=0.15,
                        help="track line width in meters")
    parser.add_argument("--inject", default=None,
                        help="world file to write the markers into")
    args = parser.parse_args()

    snippet = generate(args.tracks, args.width)
    if not snippet:
        print("no tracks or points found — nothing to generate")
        return
    if args.inject:
        inject(args.inject, snippet)
    else:
        print(snippet)


if __name__ == "__main__":
    main()
