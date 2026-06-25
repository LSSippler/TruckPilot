#!/usr/bin/env python3
"""Simulate production RouterGraph snap paths against graph.json."""
from __future__ import annotations

import json
import math
import re
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# router/src/lib.rs constants
EDGE_SNAP_RADIUS_M = 100.0
SNAP_RADIUS_M = 20.0
GOAL_SNAP_RADIUS_M = 5000.0
ENGAGE_GEOMETRIC_RADIUS_M = 50.0


def load_graph():
    g = json.loads((ROOT / "graph.json").read_text(encoding="utf-8"))
    nodes = [(int(n["uid"]), float(n["x"]), float(n["z"])) for n in g["nodes"]]
    edges_raw = g["edges"]
    edges = []
    for e in edges_raw:
        fu, tu = int(e["from"]), int(e["to"])
        dist = float(e.get("distance_m", 0))
        edges.append((fu, tu, dist))
    positions = {uid: (x, z) for uid, x, z in nodes}
    nodes_with_edges = set()
    for fu, tu, _ in edges:
        nodes_with_edges.add(fu)
        nodes_with_edges.add(tu)
    return nodes, edges, positions, nodes_with_edges


def scc_size(start_uid: int, adj, radj, uid_to_i) -> int:
    if start_uid not in uid_to_i:
        return 0
    i = uid_to_i[start_uid]
    fwd = {i}
    stack = [i]
    while stack:
        v = stack.pop()
        for w in adj[v]:
            if w not in fwd:
                fwd.add(w)
                stack.append(w)
    rev = {i}
    stack = [i]
    while stack:
        v = stack.pop()
        for w in radj[v]:
            if w not in rev:
                rev.add(w)
                stack.append(w)
    return len(fwd & rev)


def node_deg(uid, adj, radj, uid_to_i):
    i = uid_to_i[uid]
    return len(adj[i]) + len(radj[i])


def find_nearest_geometric(x, z, max_dist, nodes, nodes_with_edges):
    """plugin-api::RouterGraph::find_nearest_geometric"""
    best = None
    max_sq = max_dist * max_dist
    for uid, nx, nz in nodes:
        if uid not in nodes_with_edges:
            continue
        dx, dz = nx - x, nz - z
        d2 = dx * dx + dz * dz
        if d2 <= max_sq and (best is None or d2 < best[0]):
            best = (d2, uid, math.sqrt(d2))
    return best


def find_nearest_global_any(x, z, max_dist, nodes):
    """route_test snap_city — NO edge filter"""
    best = None
    max_sq = max_dist * max_dist
    for uid, nx, nz in nodes:
        dx, dz = nx - x, nz - z
        d2 = dx * dx + dz * dz
        if d2 <= max_sq and (best is None or d2 < best[0]):
            best = (d2, uid, math.sqrt(d2))
    return best


def find_nearest_with_heading(x, z, heading, max_dist, nodes, edges, positions, nodes_with_edges):
    """Simplified production node fallback — filters nodes_with_edges + optional heading."""
    heading_rad = -heading * math.tau
    hx = math.sin(heading_rad)
    hz = -math.cos(heading_rad)
    max_sq = max_dist * max_dist
    candidates = []
    for uid, nx, nz in nodes:
        if uid not in nodes_with_edges:
            continue
        dx, dz = nx - x, nz - z
        d2 = dx * dx + dz * dz
        if d2 <= max_sq:
            candidates.append((uid, nx, nz, math.sqrt(d2)))
    if not candidates:
        return None
    # pick nearest (heading filter skipped for conservative test — worst case still has edge filter)
    candidates.sort(key=lambda c: c[3])
    uid, _, _, dist = candidates[0]
    return uid, dist


def find_nearest_on_edge(x, z, heading, max_dist, edges, positions):
    """plugin-api::RouterGraph::find_nearest_on_edge"""
    heading_rad = -heading * math.tau
    hx = math.sin(heading_rad)
    hz = -math.cos(heading_rad)
    best_dist = float("inf")
    best_uid = None
    fallback_dist = float("inf")
    fallback_uid = None
    for fu, tu, _ in edges:
        if fu not in positions or tu not in positions:
            continue
        fx, fz = positions[fu]
        tx, tz = positions[tu]
        ex, ez = tx - fx, tz - fz
        len_sq = ex * ex + ez * ez
        if len_sq < 0.01:
            continue
        t = ((x - fx) * ex + (z - fz) * ez) / len_sq
        t = max(0.0, min(1.0, t))
        px, pz = fx + t * ex, fz + t * ez
        dist = math.hypot(x - px, z - pz)
        if dist > max_dist:
            continue
        ln = math.sqrt(len_sq)
        dot = ex / ln * hx + ez / ln * hz
        chosen = tu if dot >= 0.0 else fu
        if dist < fallback_dist:
            fallback_dist = dist
            fallback_uid = chosen
        if dot < -0.5:
            continue
        if dist < best_dist:
            best_dist = dist
            best_uid = chosen
    if best_uid is not None:
        return best_uid, best_dist, "edge"
    if fallback_uid is not None:
        return fallback_uid, fallback_dist, "edge_fallback"
    return None


def report_snap(label, uid, dist, method, uid_to_i, adj, radj, MAIN_SIZE):
    if uid is None:
        print(f"  {label}: MISS")
        return None
    deg = node_deg(uid, adj, radj, uid_to_i)
    scc = scc_size(uid, adj, radj, uid_to_i)
    in_main = scc == MAIN_SIZE
    print(
        f"  {label}: uid={uid} hex=0x{uid:016x} dist={dist:.1f}m method={method} "
        f"deg={deg} scc={scc} main={'YES' if in_main else 'NO'}"
    )
    return {"uid": uid, "deg": deg, "scc": scc, "in_main": in_main, "dist": dist, "method": method}


def read_cities():
    cities = []
    cur = {}
    for raw in (ROOT / "crates/map-parser/tests/fixtures/test_cities.toml").read_text(
        encoding="utf-8"
    ).splitlines():
        line = raw.split("#")[0].strip()
        if not line:
            continue
        if line == "[[city]]":
            if cur.get("name"):
                cities.append(cur)
            cur = {}
            continue
        m = re.match(r"^(\w+)\s*=\s*(.+)$", line)
        if m:
            k, v = m.group(1), m.group(2).strip().strip('"')
            if k == "name":
                cur["name"] = v
            elif k in ("x", "z"):
                cur[k] = float(v)
    if cur.get("name"):
        cities.append(cur)
    return cities


def main():
    nodes, edges, positions, nodes_with_edges = load_graph()
    uid_to_i = {uid: i for i, (uid, _, _) in enumerate(
        [(u, positions[u][0], positions[u][1]) for u in positions]
    )}
    # rebuild index from nodes list order
    adj = [[] for _ in range(len(nodes))]
    radj = [[] for _ in range(len(nodes))]
    uid_to_i = {}
    for i, (uid, _, _) in enumerate(nodes):
        uid_to_i[uid] = i
    for fu, tu, _ in edges:
        if fu in uid_to_i and tu in uid_to_i:
            adj[uid_to_i[fu]].append(uid_to_i[tu])
            radj[uid_to_i[tu]].append(uid_to_i[fu])

    truck_i = uid_to_i.get(3829402255816130719)
    MAIN = scc_nodes(truck_i, adj, radj) if truck_i is not None else set()
    MAIN_SIZE = len(MAIN)

    out = {"main_scc_size": MAIN_SIZE}

    print("=== SESSION CASE ===")
    truck_x, truck_z = 5345.0, 14296.0
    truck_heading = 0.75  # placeholder ETS2 east-ish
    edge = find_nearest_on_edge(truck_x, truck_z, truck_heading, EDGE_SNAP_RADIUS_M, edges, positions)
    if edge:
        start_uid, start_dist, start_method = edge
    else:
        nh = find_nearest_with_heading(
            truck_x, truck_z, truck_heading, SNAP_RADIUS_M, nodes, edges, positions, nodes_with_edges
        )
        if nh:
            start_uid, start_dist = nh
            start_method = "node"
        else:
            start_uid, start_dist, start_method = None, None, None
    out["session_start"] = report_snap(
        "Production START (edge->node fallback)", start_uid, start_dist or 0, start_method or "none",
        uid_to_i, adj, radj, MAIN_SIZE,
    )

    for gx, gz, glabel in [
        (10542.0, -10870.0, "Session set-goal-pos Berlin ref"),
        (-16400.0, -3200.0, "test_cities.toml Berlin"),
    ]:
        g = find_nearest_geometric(gx, gz, GOAL_SNAP_RADIUS_M, nodes, nodes_with_edges)
        rt = find_nearest_global_any(gx, gz, 20.0, nodes)  # route_test primary 20km? actually 20m toml
        rt20 = find_nearest_global_any(gx, gz, 20000.0, nodes)
        if g:
            out[f"goal_{glabel}"] = report_snap(
                f"Production GOAL geometric ({glabel})", g[1], g[2], "find_nearest_geometric",
                uid_to_i, adj, radj, MAIN_SIZE,
            )
            if start_uid and g[1]:
                reachable = bfs_reach(start_uid, g[1], adj, uid_to_i)
                print(f"    A* reachable from start? {reachable}")
        if rt20:
            report_snap(f"route_test-style global 20km ({glabel})", rt20[1], rt20[2], "global_no_filter",
                        uid_to_i, adj, radj, MAIN_SIZE)

    print("\n=== MAIN-SCC CITIES (production goal snap) ===")
    main_city_names = [
        "Berlin", "Hamburg", "Muenchen", "Wien", "Amsterdam", "Madrid", "Lyon", "Rom", "Mailand", "Budapest"
    ]
    cities = {c["name"]: c for c in read_cities()}
    city_results = []
    ghost_geom = 0
    non_main_geom = 0
    for name in main_city_names:
        c = cities[name]
        g = find_nearest_geometric(c["x"], c["z"], GOAL_SNAP_RADIUS_M, nodes, nodes_with_edges)
        rt = find_nearest_global_any(c["x"], c["z"], 20000.0, nodes)
        if not g:
            print(f"  {name}: production MISS")
            continue
        uid, dist = g[1], g[2]
        deg = node_deg(uid, adj, radj, uid_to_i)
        scc = scc_size(uid, adj, radj, uid_to_i)
        in_main = scc == MAIN_SIZE
        rt_uid = rt[1] if rt else None
        rt_deg = node_deg(rt_uid, adj, radj, uid_to_i) if rt_uid else -1
        print(
            f"  {name}: prod uid={uid} deg={deg} scc={scc} main={in_main} dist={dist:.0f}m | "
            f"route_test uid={rt_uid} deg={rt_deg}"
        )
        if deg == 0:
            ghost_geom += 1
        if not in_main:
            non_main_geom += 1
        city_results.append({"city": name, "uid": uid, "deg": deg, "scc": scc, "in_main": in_main})

    out["city_results"] = city_results
    out["production_goal_ghost_count"] = ghost_geom
    out["production_goal_non_main_count"] = non_main_geom

    (ROOT / "outputs/2026-06-14/routing-audit-tmp/production_snap_check.json").write_text(
        json.dumps(out, indent=2), encoding="utf-8"
    )


def scc_nodes(start_i, adj, radj):
    fwd = {start_i}
    stack = [start_i]
    while stack:
        v = stack.pop()
        for w in adj[v]:
            if w not in fwd:
                fwd.add(w)
                stack.append(w)
    rev = {start_i}
    stack = [start_i]
    while stack:
        v = stack.pop()
        for w in radj[v]:
            if w not in rev:
                rev.add(w)
                stack.append(w)
    return fwd & rev


def bfs_reach(start_uid, goal_uid, adj, uid_to_i):
    if start_uid not in uid_to_i or goal_uid not in uid_to_i:
        return False
    start, goal = uid_to_i[start_uid], uid_to_i[goal_uid]
    seen = {start}
    stack = [start]
    while stack:
        v = stack.pop()
        if v == goal:
            return True
        for w in adj[v]:
            if w not in seen:
                seen.add(w)
                stack.append(w)
    return False


if __name__ == "__main__":
    main()
