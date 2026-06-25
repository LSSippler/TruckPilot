#!/usr/bin/env python3
"""Disposable graph.json connectivity audit — outputs JSON stats to stdout."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path


def load_graph(path: Path):
    print(f"[audit] loading {path} ...", file=sys.stderr)
    with path.open("rb") as f:
        g = json.load(f)
    nodes = g["nodes"]
    edges = g["edges"]
    prefabs = g.get("prefabs", [])
    stats = g.get("stats", {})
    print(
        f"[audit] {len(nodes)} nodes, {len(edges)} edges, {len(prefabs)} prefabs",
        file=sys.stderr,
    )
    return nodes, edges, prefabs, stats


def build_indices(nodes, edges, prefabs):
    uid_to_i: dict[int, int] = {}
    coords: list[tuple[float, float, float]] = []
    for i, n in enumerate(nodes):
        uid = int(n["uid"])
        uid_to_i[uid] = i
        coords.append((float(n.get("x", 0)), float(n.get("y", 0)), float(n.get("z", 0))))

    out_deg = [0] * len(nodes)
    in_deg = [0] * len(nodes)
    adj: list[list[int]] = [[] for _ in nodes]
    edge_endpoints: list[tuple[int, int]] = []

    dangling = 0
    for e in edges:
        fu = uid_to_i.get(int(e["from"]))
        tu = uid_to_i.get(int(e["to"]))
        if fu is None or tu is None:
            dangling += 1
            continue
        out_deg[fu] += 1
        in_deg[tu] += 1
        adj[fu].append(tu)
        edge_endpoints.append((fu, tu))

    prefab_uids: set[int] = set()
    for p in prefabs:
        for u in p.get("connected_node_uids", []) or []:
            prefab_uids.add(int(u))

    return uid_to_i, coords, out_deg, in_deg, adj, edge_endpoints, prefab_uids, dangling


def degree_histogram(out_deg, in_deg):
    total = [o + i for o, i in zip(out_deg, in_deg)]
    hist = Counter(total)
    iso = hist.get(0, 0)
    deg1 = hist.get(1, 0)
    return {
        "histogram_total_degree": {str(k): hist[k] for k in sorted(hist.keys())},
        "degree_0": iso,
        "degree_1": deg1,
        "degree_2": hist.get(2, 0),
        "degree_3": hist.get(3, 0),
        "degree_4_plus": sum(v for k, v in hist.items() if k >= 4),
        "degree_0_pct": iso / len(total) * 100 if total else 0,
        "isolated_directed": sum(
            1 for o, i in zip(out_deg, in_deg) if o == 0 and i == 0
        ),
        "source_only": sum(1 for o, i in zip(out_deg, in_deg) if o > 0 and i == 0),
        "sink_only": sum(1 for o, i in zip(out_deg, in_deg) if o == 0 and i > 0),
    }


def bfs_reachable(adj: list[list[int]], start: int) -> set[int]:
    seen = {start}
    stack = [start]
    while stack:
        v = stack.pop()
        for w in adj[v]:
            if w not in seen:
                seen.add(w)
                stack.append(w)
    return seen


def mutual_reachability(adj, a: int, b: int) -> dict:
    if a == b:
        return {"mutually_reachable": True, "a_to_b": True, "b_to_a": True}
    reach_a = bfs_reachable(adj, a)
    reach_b = bfs_reachable(adj, b)
    a_to_b = b in reach_a
    b_to_a = a in reach_b
    return {
        "mutually_reachable": a_to_b and b_to_a,
        "a_to_b": a_to_b,
        "b_to_a": b_to_a,
    }


def spatial_bins(coords, mask, cell=4096.0):
    bins: Counter[tuple[int, int]] = Counter()
    for i, m in enumerate(mask):
        if not m:
            continue
        x, _, z = coords[i]
        bins[(int(x // cell), int(z // cell))] += 1
    return bins


def nearest_uid(uid_to_i, coords, x, z, max_m=5000.0):
    best = None
    best_d2 = max_m * max_m
    for uid, i in uid_to_i.items():
        cx, _, cz = coords[i]
        d2 = (cx - x) ** 2 + (cz - z) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = uid
    if best is None:
        return None, None
    return best, math.sqrt(best_d2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    ap.add_argument("--out", default="outputs/2026-06-14/graph-audit-tmp/audit_extra.json")
    ap.add_argument("--islands-json", default="outputs/2026-06-14/graph-audit-tmp/routing_islands.json")
    args = ap.parse_args()

    nodes, edges, prefabs, build_stats = load_graph(Path(args.graph))
    uid_to_i, coords, out_deg, in_deg, adj, edge_eps, prefab_uids, dangling = build_indices(
        nodes, edges, prefabs
    )

    deg = degree_histogram(out_deg, in_deg)

    iso_mask = [o == 0 and i == 0 for o, i in zip(out_deg, in_deg)]
    iso_prefab = sum(1 for i, m in enumerate(iso_mask) if m and int(nodes[i]["uid"]) in prefab_uids)
    iso_non_prefab = deg["isolated_directed"] - iso_prefab

    iso_bins = spatial_bins(coords, iso_mask, cell=4096.0)
    connected_mask = [not m for m in iso_mask]
    conn_bins = spatial_bins(coords, connected_mask, cell=4096.0)

    islands_path = Path(args.islands_json)
    islands = json.loads(islands_path.read_text(encoding="utf-8")) if islands_path.exists() else {}
    scc_summary = {
        "source": str(islands_path),
        "total_sccs": islands.get("total_sccs"),
        "singleton_sccs": islands.get("singleton_sccs"),
        "largest_size": islands["top_components"][0]["size"] if islands.get("top_components") else None,
        "largest_pct": (
            islands["top_components"][0]["size"] / len(nodes) * 100
            if islands.get("top_components")
            else None
        ),
        "second_size": (
            islands["top_components"][1]["size"]
            if islands.get("top_components") and len(islands["top_components"]) > 1
            else None
        ),
        "cities_in_largest": islands.get("cities_in_largest"),
        "city_total": islands.get("city_total"),
    }

    # Case study coords / uids
    truck_x, truck_z = 5345.0, 14282.0
    plus_z_x, plus_z_z = 5350.0, 16500.0
    routable_minus_z_uid = 4809608989288380432
    snap_edge_uid = 3829402255816130719

    truck_uid, truck_snap = nearest_uid(uid_to_i, coords, truck_x, truck_z)
    plus_uid, plus_snap = nearest_uid(uid_to_i, coords, plus_z_x, plus_z_z)

    def node_report(uid: int | None):
        if uid is None or uid not in uid_to_i:
            return {"uid": uid, "found": False}
        i = uid_to_i[uid]
        x, y, z = coords[i]
        return {
            "uid": uid,
            "uid_hex": f"0x{uid:016x}",
            "found": True,
            "x": x,
            "y": y,
            "z": z,
            "out_deg": out_deg[i],
            "in_deg": in_deg[i],
            "total_deg": out_deg[i] + in_deg[i],
            "in_prefab_clique": uid in prefab_uids,
        }

    truck_i = uid_to_i.get(truck_uid) if truck_uid else None
    plus_i = uid_to_i.get(plus_uid) if plus_uid else None
    minus_i = uid_to_i.get(routable_minus_z_uid)

    reach = {}
    if truck_i is not None and plus_i is not None:
        reach["truck_vs_plus_z"] = mutual_reachability(adj, truck_i, plus_i)
    if truck_i is not None and minus_i is not None:
        reach["truck_vs_routable_minus_z"] = mutual_reachability(adj, truck_i, minus_i)
    if plus_i is not None and minus_i is not None:
        reach["plus_z_vs_routable_minus_z"] = mutual_reachability(adj, plus_i, minus_i)

    case = {
        "truck_position": {"x": truck_x, "z": truck_z},
        "truck_nearest_node": node_report(truck_uid) | {"snap_m": truck_snap},
        "plus_z_goal_position": {"x": plus_z_x, "z": plus_z_z},
        "plus_z_nearest_node": node_report(plus_uid) | {"snap_m": plus_snap},
        "routable_minus_z_uid": node_report(routable_minus_z_uid),
        "snap_edge_uid_as_node": node_report(snap_edge_uid),
        "mutual_reachability": reach,
        "same_scc_truck_vs_plus_z": reach.get("truck_vs_plus_z", {}).get("mutually_reachable"),
        "same_scc_truck_vs_routable_minus_z": reach.get("truck_vs_routable_minus_z", {}).get(
            "mutually_reachable"
        ),
    }

    # Edge lookup for snap_edge_uid
    edge_hits = [
        {
            "from": int(e["from"]),
            "to": int(e["to"]),
            "direction": e.get("direction"),
            "uid": int(e.get("uid", 0)),
        }
        for e in edges
        if int(e.get("uid", 0)) == snap_edge_uid
        or int(e["from"]) == snap_edge_uid
        or int(e["to"]) == snap_edge_uid
    ][:5]
    case["snap_edge_lookup"] = edge_hits[:5]

    out = {
        "node_count": len(nodes),
        "edge_count": len(edges),
        "dangling_edge_refs": dangling,
        "build_stats": build_stats,
        "degree": deg,
        "isolated_breakdown": {
            "total_isolated": deg["isolated_directed"],
            "in_prefab_clique": iso_prefab,
            "not_in_prefab_clique": iso_non_prefab,
            "isolated_with_coords_nonzero": sum(
                1
                for i, m in enumerate(iso_mask)
                if m and (coords[i][0] != 0 or coords[i][2] != 0)
            ),
        },
        "spatial_iso_top10_bins_4km": [[list(k), v] for k, v in iso_bins.most_common(10)],
        "spatial_connected_top5_bins_4km": [[list(k), v] for k, v in conn_bins.most_common(5)],
        "scc": scc_summary,
        "case_study_no_path": case,
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(f"[audit] wrote {out_path}", file=sys.stderr)
    print(json.dumps({k: out[k] for k in ("node_count", "degree", "scc", "case_study_no_path")}, indent=2))


if __name__ == "__main__":
    main()
