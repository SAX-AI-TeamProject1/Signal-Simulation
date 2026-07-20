#!/usr/bin/env python3
# Last updated: 2026-07-20
"""Generate Gazebo <actor> loops from tracks.yaml.

Reads the tracks.yaml produced by tools/track_editor.py and emits one scripted
<actor> per loop track (Method A in docs/transport-track-guide.md). Non-loop
tracks (e.g. the approach track) are skipped — those belong to real vehicles.

Usage:
    # print the actor SDF snippet
    python3 tools/tracks_to_actors.py --tracks config/tracks.yaml

    # write/replace the actors inside a world file, between the markers
    # <!-- BEGIN GENERATED ACTORS --> ... <!-- END GENERATED ACTORS -->
    # (markers are added before </world> if missing)
    python3 tools/tracks_to_actors.py --tracks config/tracks.yaml \
        --inject worlds/navi_factory/world/navi_factory/navi_factory.sdf
"""

import argparse
import math
import pathlib
import re

import yaml

BEGIN_MARK = "<!-- BEGIN GENERATED ACTORS -->"
END_MARK = "<!-- END GENERATED ACTORS -->"

BOX_VISUAL = """\
      <visual name="visual">
        <geometry>
          <box><size>1.2 0.8 0.5</size></box>
        </geometry>
      </visual>"""

MESH_VISUAL = """\
      <visual name="visual">
        <geometry>
          <mesh><uri>{mesh}</uri></mesh>
        </geometry>
      </visual>"""


def build_waypoints(waypoints, loop, speed, turn_time, z):
    """Return (time, x, y, yaw) tuples for the actor script."""
    points = [(wp[0], wp[1], wp[2]) for wp in waypoints]
    if loop:
        points.append(points[0])
    out = []
    t = 0.0
    for i, (x, y, yaw) in enumerate(points):
        if i > 0:
            px, py, pyaw = points[i - 1]
            t += math.hypot(x - px, y - py) / speed
            out.append((t, x, y, pyaw))  # arrive still facing the old heading
            if abs(yaw - pyaw) > 1e-3:
                t += turn_time
        out.append((t, x, y, yaw))
    return [(round(t, 2), x, y, yaw) for t, x, y, yaw in out]


def actor_sdf(name, waypoints, loop, speed, turn_time, z, mesh):
    visual = MESH_VISUAL.format(mesh=mesh) if mesh else BOX_VISUAL
    lines = [
        f'<actor name="{name}">',
        '  <link name="body">',
        visual,
        "  </link>",
        "  <script>",
        "    <loop>true</loop>",
        "    <auto_start>true</auto_start>",
        '    <trajectory id="0" type="square">',
    ]
    for t, x, y, yaw in build_waypoints(waypoints, loop, speed, turn_time, z):
        lines.append(
            f"      <waypoint><time>{t}</time>"
            f"<pose>{x} {y} {z} 0 0 {yaw}</pose></waypoint>"
        )
    lines += ["    </trajectory>", "  </script>", "</actor>"]
    return "\n".join(lines)


def generate(tracks_path, speed, turn_time, z, mesh):
    with open(tracks_path) as f:
        data = yaml.safe_load(f)
    actors = []
    for name, track in (data.get("tracks") or {}).items():
        if not track.get("loop"):
            print(f"skipping non-loop track: {name}")
            continue
        if len(track.get("waypoints", [])) < 2:
            print(f"skipping track with <2 waypoints: {name}")
            continue
        actors.append(
            actor_sdf(f"{name}_vehicle", track["waypoints"], True,
                      speed, turn_time, z, mesh)
        )
    return "\n".join(actors)


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
    print(f"injected actors into {world_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracks", default="config/tracks.yaml")
    parser.add_argument("--speed", type=float, default=0.75, help="m/s")
    parser.add_argument("--turn-time", type=float, default=1.0,
                        help="seconds spent rotating at each corner")
    parser.add_argument("--z", type=float, default=0.0, help="actor height offset")
    parser.add_argument("--mesh", default=None,
                        help="visual mesh uri (default: plain box)")
    parser.add_argument("--inject", default=None,
                        help="world file to write the actors into")
    args = parser.parse_args()

    snippet = generate(args.tracks, args.speed, args.turn_time, args.z, args.mesh)
    if not snippet:
        print("no loop tracks found — nothing to generate")
        return
    if args.inject:
        inject(args.inject, snippet)
    else:
        print(snippet)


if __name__ == "__main__":
    main()
