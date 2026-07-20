#!/usr/bin/env python3
# Last updated: 2026-07-20
"""Click-based track editor.

Opens an occupancy map (map.yaml + image) with matplotlib and lets you place
track waypoints by clicking. Saves/loads the tracks.yaml format described in
docs/transport-track-guide.md (yaw is computed automatically from the segment
direction, so you only click x/y positions).

Usage:
    python3 tools/track_editor.py \
        --map <path-to-map.yaml> \
        --out config/tracks.yaml

Controls (in the plot window):
    left click   add waypoint to the active track
    right click  remove last waypoint of the active track
    n            start a new track
    t            cycle active track
    l            toggle loop on the active track
    p            place/move the signal point at the cursor
    m            add a station point at the cursor (station_1, station_2, ...)
    d            delete the station point nearest to the cursor
    s            save tracks.yaml
"""

import argparse
import math
import pathlib

import matplotlib.pyplot as plt
import yaml
from PIL import Image


def load_map(map_yaml_path):
    map_yaml_path = pathlib.Path(map_yaml_path)
    with open(map_yaml_path) as f:
        info = yaml.safe_load(f)
    image = Image.open(map_yaml_path.parent / info["image"])
    resolution = float(info["resolution"])
    origin_x, origin_y = float(info["origin"][0]), float(info["origin"][1])
    width_m = image.width * resolution
    height_m = image.height * resolution
    # imshow with origin="upper": image row 0 (top) is the max-y edge of the map.
    extent = [origin_x, origin_x + width_m, origin_y, origin_y + height_m]
    return image, extent


def compute_yaws(points, loop):
    """Yaw of each waypoint = direction toward the next waypoint."""
    yaws = []
    n = len(points)
    for i in range(n):
        if i < n - 1:
            nxt = points[i + 1]
        elif loop:
            nxt = points[0]
        else:
            yaws.append(yaws[-1] if yaws else 0.0)
            break
        dx, dy = nxt[0] - points[i][0], nxt[1] - points[i][1]
        yaws.append(round(math.atan2(dy, dx), 4))
    return yaws


class TrackEditor:
    def __init__(self, map_yaml, out_path):
        self.out_path = pathlib.Path(out_path)
        self.signal_point = None  # [x, y, yaw]
        self.stations = {}  # name -> [x, y, yaw]
        self.tracks = {}  # name -> {"loop": bool, "points": [[x, y], ...]}
        self._load_existing()
        if not self.tracks:
            self.tracks["track_1"] = {"loop": True, "points": []}
        self.active = next(iter(self.tracks))

        image, self.extent = load_map(map_yaml)
        self.fig, self.ax = plt.subplots(figsize=(8, 10))
        self.ax.imshow(image, cmap="gray", extent=self.extent)
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.artists = []
        self.redraw()

    def _load_existing(self):
        if not self.out_path.exists():
            return
        with open(self.out_path) as f:
            data = yaml.safe_load(f) or {}
        self.signal_point = data.get("signal_point")
        self.stations = dict(data.get("stations") or {})
        for name, track in (data.get("tracks") or {}).items():
            points = [[wp[0], wp[1]] for wp in track.get("waypoints", [])]
            self.tracks[name] = {"loop": bool(track.get("loop")), "points": points}
        print(f"loaded {len(self.tracks)} track(s) from {self.out_path}")

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        points = self.tracks[self.active]["points"]
        if event.button == 1:
            points.append([round(event.xdata, 3), round(event.ydata, 3)])
        elif event.button == 3 and points:
            points.pop()
        self.redraw()

    def on_key(self, event):
        if event.key == "n":
            name = f"track_{len(self.tracks) + 1}"
            self.tracks[name] = {"loop": True, "points": []}
            self.active = name
        elif event.key == "t":
            names = list(self.tracks)
            self.active = names[(names.index(self.active) + 1) % len(names)]
        elif event.key == "l":
            self.tracks[self.active]["loop"] = not self.tracks[self.active]["loop"]
        elif event.key in ("p", "m", "d"):
            if event.inaxes != self.ax or event.xdata is None:
                return
            x, y = round(event.xdata, 3), round(event.ydata, 3)
            if event.key == "p":
                yaw = self.signal_point[2] if self.signal_point else 0.0
                self.signal_point = [x, y, yaw]
            elif event.key == "m":
                i = 1
                while f"station_{i}" in self.stations:
                    i += 1
                self.stations[f"station_{i}"] = [x, y, 0.0]
            elif self.stations:  # "d"
                nearest = min(
                    self.stations,
                    key=lambda n: math.hypot(self.stations[n][0] - x,
                                             self.stations[n][1] - y),
                )
                del self.stations[nearest]
        elif event.key == "s":
            self.save()
        self.redraw()

    def redraw(self):
        for artist in self.artists:
            artist.remove()
        self.artists = []
        for name, track in self.tracks.items():
            points = track["points"]
            if not points:
                continue
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            if track["loop"] and len(points) > 2:
                xs, ys = xs + [xs[0]], ys + [ys[0]]
            is_active = name == self.active
            (line,) = self.ax.plot(
                xs, ys, "o-",
                linewidth=2 if is_active else 1,
                markersize=6 if is_active else 4,
                alpha=1.0 if is_active else 0.5,
                label=name,
            )
            self.artists.append(line)
            self.artists.append(
                self.ax.annotate("0", points[0], color=line.get_color(), fontsize=9)
            )
        if self.signal_point:
            x, y = self.signal_point[0], self.signal_point[1]
            (star,) = self.ax.plot(x, y, "r*", markersize=15)
            self.artists.append(star)
            self.artists.append(
                self.ax.annotate("signal", (x, y), color="red", fontsize=9)
            )
        for name, (x, y, _yaw) in self.stations.items():
            (tri,) = self.ax.plot(x, y, "g^", markersize=10)
            self.artists.append(tri)
            self.artists.append(
                self.ax.annotate(name, (x, y), color="green", fontsize=8)
            )
        if self.tracks and any(t["points"] for t in self.tracks.values()):
            self.artists.append(self.ax.legend(loc="upper right"))
        track = self.tracks[self.active]
        self.ax.set_title(
            f"active: {self.active}  (loop={track['loop']}, "
            f"{len(track['points'])} wp)\n"
            "click=add  right-click=undo  n=new  t=switch  l=loop\n"
            "p=signal point  m=add station  d=del station  s=save"
        )
        self.fig.canvas.draw_idle()

    def save(self):
        data = {}
        if self.signal_point is not None:
            data["signal_point"] = self.signal_point
        if self.stations:
            data["stations"] = self.stations
        data["tracks"] = {}
        for name, track in self.tracks.items():
            points = track["points"]
            if len(points) < 2:
                continue
            yaws = compute_yaws(points, track["loop"])
            data["tracks"][name] = {
                "loop": track["loop"],
                "waypoints": [[p[0], p[1], yaw] for p, yaw in zip(points, yaws)],
            }
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.out_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
        print(f"saved {len(data['tracks'])} track(s) to {self.out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, help="path to map.yaml")
    parser.add_argument("--out", default="config/tracks.yaml", help="tracks.yaml path")
    args = parser.parse_args()
    TrackEditor(args.map, args.out)
    plt.show()


if __name__ == "__main__":
    main()
