"""Validate the LIF map against the assignment rules and the SLAM occupancy map.

Checks:
  * PickupStation and DropoffStation exist and point at real nodes
  * every edge references real nodes, and an edge path joins pickup to dropoff
  * each node sits on free floor with enough clearance for the robot

Usage:  python tools/check_lif.py
"""
from __future__ import annotations

import json
import math
import sys
from collections import deque
from pathlib import Path

import yaml  # pip install pyyaml

MAPS = Path(__file__).resolve().parent.parent / "maps"
LIF_PATH = MAPS / "house.lif.json"
MAP_YAML = MAPS / "house_map.yaml"

ROBOT_RADIUS = 0.15   # turtlebot3 waffle.yaml robot_radius (hard limit)
COMFORT = 0.5         # waffle.yaml inflation_radius (costs rise inside this)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    width, height = int(tokens[1]), int(tokens[2])
    return width, height, data[i + 1:i + 1 + width * height]


def wall_distance(width: int, height: int, pixels: bytes) -> list[int]:
    """Pixel distance from every cell to the nearest wall (8-neighbour BFS)."""
    dist = [10**9] * (width * height)
    queue = deque(k for k in range(width * height) if pixels[k] < 100)
    for k in queue:
        dist[k] = 0
    while queue:
        k = queue.popleft()
        r, c = divmod(k, width)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rr, cc = r + dr, c + dc
                if 0 <= rr < height and 0 <= cc < width:
                    kk = rr * width + cc
                    if dist[kk] > dist[k] + 1:
                        dist[kk] = dist[k] + 1
                        queue.append(kk)
    return dist


def main() -> int:
    meta = yaml.safe_load(MAP_YAML.read_text())
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    width, height, pixels = read_pgm(MAPS / meta["image"])
    dist = wall_distance(width, height, pixels)

    layout = json.loads(LIF_PATH.read_text())["layouts"][0]
    nodes = {n["nodeId"]: n["nodePosition"] for n in layout["nodes"]}
    problems: list[str] = []
    warnings: list[str] = []

    print(f"{len(nodes)} nodes, {len(layout['edges'])} edges, {len(layout['stations'])} stations\n")
    for node_id, pos in nodes.items():
        col = int((pos["x"] - ox) / res)
        row = height - 1 - int((pos["y"] - oy) / res)
        if not (0 <= col < width and 0 <= row < height):
            problems.append(f"{node_id} is outside the map image")
            continue
        value, clearance = pixels[row * width + col], dist[row * width + col] * res
        kind = "free" if value > 250 else "WALL" if value < 100 else "unscanned"
        print(f"  {node_id:<5} ({pos['x']:6.2f}, {pos['y']:6.2f})  {kind:<9} clearance {clearance:.2f} m")
        if kind == "WALL" or clearance < ROBOT_RADIUS:
            problems.append(f"{node_id} is on or inside a wall")
        elif kind == "unscanned":
            warnings.append(f"{node_id} is on unscanned floor (reachable only if the robot can drive there)")
        elif clearance < COMFORT:
            warnings.append(f"{node_id} is {clearance:.2f} m from a wall (fine, but Nav2 costs rise)")

    successors: dict[str, set[str]] = {n: set() for n in nodes}
    for edge in layout["edges"]:
        start, end = edge["startNodeId"], edge["endNodeId"]
        if start in nodes and end in nodes:
            successors[start].add(end)
        else:
            problems.append(f"edge {edge['edgeId']} references a missing node")

    stations = {s["stationName"]: s["interactionNodeIds"] for s in layout["stations"]}
    print()
    for name in ("PickupStation", "DropoffStation"):
        ids = stations.get(name)
        print(f"  {name}: {ids}")
        if not ids or ids[0] not in nodes:
            problems.append(f"{name} missing or not linked to a real node")

    if not problems:
        pickup, dropoff = stations["PickupStation"][0], stations["DropoffStation"][0]
        previous: dict[str, str | None] = {pickup: None}
        queue = deque([pickup])
        while queue:
            current = queue.popleft()
            for nxt in successors[current] - previous.keys():
                previous[nxt] = current
                queue.append(nxt)
        if dropoff not in previous:
            problems.append("no edge path from PickupStation to DropoffStation")
        else:
            path, node = [], dropoff
            while node is not None:
                path.append(node)
                node = previous[node]
            print(f"\n  route pickup -> dropoff: {' -> '.join(reversed(path))}")

    print("\nPROBLEMS:", *(problems or ["none"]), sep="\n  ")
    print("WARNINGS:", *(warnings or ["none"]), sep="\n  ")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
