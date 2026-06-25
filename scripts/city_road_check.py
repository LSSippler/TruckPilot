#!/usr/bin/env python3
"""Check city road node availability - quick analysis."""
import json, re, numpy as np
from scipy.spatial import cKDTree

with open("graph.json", "rb") as f:
    g = json.load(f)
nodes = g["nodes"]
N = len(nodes)

# Identify road nodes
edge_froms = set()
edge_tos = set()
for e in g["edges"]:
    edge_froms.add(int(e["from"]))
    edge_tos.add(int(e["to"]))
all_edge_nodes = edge_froms | edge_tos

# Load cities
def read_cities(path):
    cities = []
    cur = {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            if line == "[[city]]":
                if cur and "name" in cur:
                    cities.append(cur)
                cur = {}
                continue
            m = re.match(r"^(\w+)\s*=\s*(.+)$", line)
            if m:
                k, vraw = m.group(1), m.group(2).strip().strip('"')
                if k == "name":
                    cur["name"] = vraw
                elif k in ("x", "z"):
                    cur[k] = float(vraw)
    if cur and "name" in cur:
        cities.append(cur)
    return cities

cities = read_cities("crates/map-parser/tests/fixtures/test_cities.toml")
print(f"Total cities: {len(cities)}")

uid_to_i = {}
for i, n in enumerate(nodes):
    uid_to_i[int(n["uid"])] = i

coords = np.zeros((N, 3), dtype=np.float64)
for i, n in enumerate(nodes):
    coords[i] = (float(n["x"]), float(n["y"]), float(n["z"]))

has_edge = np.zeros(N, dtype=bool)
for i, n in enumerate(nodes):
    if int(n["uid"]) in all_edge_nodes:
        has_edge[i] = True

# KD-tree for road nodes only
road_mask = has_edge
road_coords = coords[road_mask]
road_global_idx = np.where(road_mask)[0]
tree_road = cKDTree(np.column_stack([road_coords[:, 0], road_coords[:, 2]]))

# KD-tree for ALL nodes
all_pts = np.column_stack([coords[:, 0], coords[:, 2]])
tree_all = cKDTree(all_pts)

print()
print(f"{'City':<15} {'AllSnap':>8} {'RoadSnap':>9} {'Roads1km':>9} {'Status'}")
print("-" * 55)

missing_road = []
found_road = []

for city in cities:
    name = city["name"]
    cx, cz = city["x"], city["z"]

    # Nearest ANY node
    d_any, _ = tree_all.query([[cx, cz]], k=1)
    d_any = float(np.ravel(d_any)[0])

    # Nearest ROAD node
    d_road, idx = tree_road.query([[cx, cz]], k=1)
    d_road = float(np.ravel(d_road)[0])
    ri = int(np.ravel(idx)[0])
    road_ni = int(road_global_idx[ri])

    # Road nodes within 1km
    nearby = tree_road.query_ball_point([cx, cz], 1000.0)

    if d_road > 1000:
        status = "NO_ROAD_NODE"
        missing_road.append(name)
    elif d_road > 200:
        status = "FAR_ROAD_NODE"
        found_road.append(name)
    else:
        status = "OK"
        found_road.append(name)

    print(f"{name:<15} {d_any:>8.1f} {d_road:>9.1f} {len(nearby):>9} {status}")

print()
print(f"Cities with road node <200m: {len(found_road)}")
print(f"Cities with road node >1km: {len(missing_road)}")
print(f"Missing road: {', '.join(missing_road)}")
print()
print(f"Found road: {', '.join(found_road)}")
