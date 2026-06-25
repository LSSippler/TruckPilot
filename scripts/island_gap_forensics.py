#!/usr/bin/env python3
"""Forensics: why boundary stitch did not bridge truck SCC <-> 9-node island."""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict, deque
from pathlib import Path

# spatial_match.rs — exact stitch constants
BOUNDARY_STITCH_MAX_DIST_M = 50.0
BOUNDARY_STITCH_HEADING_DOT_MIN = 0.7
BOUNDARY_STITCH_Z_TOL_M = 5.0
DEFAULT_CELL_SIZE = 250.0
VIRTUAL_SECTOR_SIZE = 4096.0
BOUNDARY_STITCH_DIRECTION = "cross_sector_boundary"

TRUCK_UID = 3829402255816130719
ISLAND_UIDS = [
    4809608987929425903,
    4809608988646651886,
    4809608989288380432,
    4809608987367386942,
    4809608988529209149,
    4809608986817933169,
    4809608987824564834,
    4809608988835392097,
    4809608987040230046,
]


def load_graph(path: Path):
    with path.open("rb") as f:
        g = json.load(f)
    return g["nodes"], g["edges"]


def build_indices(nodes, edges):
    uid_to_i = {int(n["uid"]): i for i, n in enumerate(nodes)}
    coords = [(float(n["x"]), float(n["y"]), float(n["z"])) for n in nodes]
    adj: list[list[int]] = [[] for _ in nodes]
    radj: list[list[int]] = [[] for _ in nodes]
    edge_pairs: set[tuple[int, int]] = set()
    dirs: dict[int, set[str]] = defaultdict(set)
    for e in edges:
        fu = uid_to_i.get(int(e["from"]))
        tu = uid_to_i.get(int(e["to"]))
        if fu is None or tu is None:
            continue
        adj[fu].append(tu)
        radj[tu].append(fu)
        d = str(e.get("direction", ""))
        dirs[int(e["from"])].add(d)
        dirs[int(e["to"])].add(d)
        a, b = sorted((fu, tu))
        edge_pairs.add((a, b))
    return uid_to_i, coords, adj, radj, edge_pairs, dirs


def scc_of(start: int, adj, radj) -> set[int]:
    fwd = {start}
    q = deque([start])
    while q:
        v = q.popleft()
        for w in adj[v]:
            if w not in fwd:
                fwd.add(w)
                q.append(w)
    rev = {start}
    q = deque([start])
    while q:
        v = q.popleft()
        for w in radj[v]:
            if w not in rev:
                rev.add(w)
                q.append(w)
    return fwd & rev


def dist3(a, b) -> tuple[float, float]:
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    xz = math.sqrt(dx * dx + dz * dz)
    return math.sqrt(dx * dx + dy * dy + dz * dz), xz


def virtual_sector(x: float, z: float) -> tuple[int, int]:
    return int(x // VIRTUAL_SECTOR_SIZE), int(z // VIRTUAL_SECTOR_SIZE)


def uid_high32(uid: int) -> int:
    return uid >> 32


def road_tangents_from_graph(
    uid: int,
    uid_to_i,
    coords,
    edges_list,
    dirs,
) -> list[tuple[float, float, float]]:
    """Approximate road tangents using forward/backward edges only."""
    i = uid_to_i[uid]
    tangents = []
    seen = set()
    for e in edges_list:
        fr, to = int(e["from"]), int(e["to"])
        d = str(e.get("direction", ""))
        if d not in {"forward", "backward", "bidirectional_unknown"}:
            continue
        if fr == uid and to in uid_to_i:
            j = uid_to_i[to]
            key = (i, j)
            if key in seen:
                continue
            seen.add(key)
            a, b = coords[i], coords[j]
        elif to == uid and fr in uid_to_i:
            j = uid_to_i[fr]
            key = (i, j)
            if key in seen:
                continue
            seen.add(key)
            a, b = coords[i], coords[j]
        else:
            continue
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        ln = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ln < 0.001:
            continue
        tangents.append((dx / ln, dy / ln, dz / ln))
    return tangents


def max_tangent_dot(a: list, b: list) -> float | None:
    if not a or not b:
        return None
    best = -math.inf
    for ta in a:
        for tb in b:
            dot = ta[0] * tb[0] + ta[1] * tb[1] + ta[2] * tb[2]
            best = max(best, dot)
    return best


def query_circle_xz(cx, cz, radius, uid_to_i, coords):
    """Mirror spatial_match::query_circle (2D XZ)."""
    cell_radius = int(math.ceil(radius / DEFAULT_CELL_SIZE)) + 1
    ccx = int(cx // DEFAULT_CELL_SIZE)
    ccz = int(cz // DEFAULT_CELL_SIZE)
    r2 = radius * radius
    hits = []
    for dix in range(-cell_radius, cell_radius + 1):
        for diz in range(-cell_radius, cell_radius + 1):
            # brute over all nodes in cell bucket
            pass
    # brute all nodes — graph size ok for forensic subset check
    for uid, i in uid_to_i.items():
        x, _, z = coords[i]
        dx, dz = x - cx, z - cz
        if dx * dx + dz * dz <= r2:
            hits.append(uid)
    return hits


def is_road_endpoint_proxy(uid: int, dirs) -> bool:
    d = dirs.get(uid, set())
    return bool(d & {"forward", "backward", "bidirectional_unknown"})


def stitch_pair_verdict(
    uid_a: int,
    uid_b: int,
    uid_to_i,
    coords,
    edges_list,
    edge_pairs,
    dirs,
    merge_sector_a: int | None,
    merge_sector_b: int | None,
) -> dict:
    """Run one ordered pair through boundary-stitch gates (mirrors spatial_match.rs)."""
    ia, ib = uid_to_i[uid_a], uid_to_i[uid_b]
    pa, pb = coords[ia], coords[ib]
    d3, dxz = dist3(pa, pb)

    out = {
        "uid_a": uid_a,
        "uid_b": uid_b,
        "dist_3d_m": d3,
        "dist_xz_m": dxz,
        "dy_m": abs(pb[1] - pa[1]),
        "virtual_sector_a": virtual_sector(pa[0], pa[2]),
        "virtual_sector_b": virtual_sector(pb[0], pb[2]),
        "uid_high32_a": f"0x{uid_high32(uid_a):08x}",
        "uid_high32_b": f"0x{uid_high32(uid_b):08x}",
        "merge_sector_a": merge_sector_a,
        "merge_sector_b": merge_sector_b,
        "in_spatial_query_a_to_b": uid_b
        in set(query_circle_xz(pa[0], pa[2], BOUNDARY_STITCH_MAX_DIST_M, uid_to_i, coords)),
        "in_spatial_query_b_to_a": uid_a
        in set(query_circle_xz(pb[0], pb[2], BOUNDARY_STITCH_MAX_DIST_M, uid_to_i, coords)),
        "road_endpoint_a": is_road_endpoint_proxy(uid_a, dirs),
        "road_endpoint_b": is_road_endpoint_proxy(uid_b, dirs),
    }

    reasons = []

    if not out["road_endpoint_a"] or not out["road_endpoint_b"]:
        reasons.append("not_road_endpoint")

    if not out["in_spatial_query_a_to_b"] and not out["in_spatial_query_b_to_a"]:
        reasons.append("never_candidate_radius")
        out["verdict_bucket"] = "never_candidate_radius"
        out["reasons"] = reasons
        return out

    # simulate loop from uid_a as anchor (as stitch pass would when iterating a)
    if merge_sector_a is not None and merge_sector_b is not None:
        if merge_sector_a == merge_sector_b:
            reasons.append("same_sector")
    elif uid_high32(uid_a) == uid_high32(uid_b):
        reasons.append("same_sector_uid_family")

    pair = tuple(sorted((ia, ib)))
    if pair in edge_pairs:
        reasons.append("already_connected")

    if out["dy_m"] >= BOUNDARY_STITCH_Z_TOL_M:
        reasons.append("distance_z_tol")
    if d3 > BOUNDARY_STITCH_MAX_DIST_M:
        reasons.append("distance_3d")

    ta = road_tangents_from_graph(uid_a, uid_to_i, coords, edges_list, dirs)
    tb = road_tangents_from_graph(uid_b, uid_to_i, coords, edges_list, dirs)
    dot = max_tangent_dot(ta, tb)
    out["tangent_count_a"] = len(ta)
    out["tangent_count_b"] = len(tb)
    out["best_heading_dot"] = dot
    if not ta or not tb:
        reasons.append("no_heading")
    elif dot is not None and dot < BOUNDARY_STITCH_HEADING_DOT_MIN:
        reasons.append("heading")

    if "never_candidate_radius" not in reasons:
        if "same_sector" in reasons or "same_sector_uid_family" in reasons:
            out["verdict_bucket"] = "same_sector"
        elif "already_connected" in reasons:
            out["verdict_bucket"] = "already_connected"
        elif "distance_z_tol" in reasons or "distance_3d" in reasons:
            out["verdict_bucket"] = "distance"
        elif "no_heading" in reasons:
            out["verdict_bucket"] = "no_heading"
        elif "heading" in reasons:
            out["verdict_bucket"] = "heading"
        else:
            out["verdict_bucket"] = "would_match"

    out["reasons"] = reasons
    return out


def nearest_pairs(island_uids, main_scc, uid_to_i, coords, node_uids):
    pairs = []
    for iu in island_uids:
        ii = uid_to_i[iu]
        pa = coords[ii]
        best = None
        for ji in main_scc:
            pb = coords[ji]
            d3, dxz = dist3(pa, pb)
            if best is None or d3 < best[0]:
                best = (d3, dxz, node_uids[ji], pb, ji)
        pairs.append((iu, pa, best))
    pairs.sort(key=lambda x: x[2][0])
    return pairs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    ap.add_argument("--out", default="outputs/2026-06-14/graph-audit-tmp/island_gap_forensics.json")
    args = ap.parse_args()

    nodes, edges = load_graph(Path(args.graph))
    uid_to_i, coords, adj, radj, edge_pairs, dirs = build_indices(nodes, edges)

    truck_i = uid_to_i[TRUCK_UID]
    main_scc = scc_of(truck_i, adj, radj)

    node_uids = [int(n["uid"]) for n in nodes]
    top_pairs = nearest_pairs(ISLAND_UIDS, main_scc, uid_to_i, coords, node_uids)

    # merge sector proxy: uid high32 family groups nodes from same .base emission
    def merge_sector_proxy(uid: int) -> int:
        return uid_high32(uid)

    best_iu, best_pa, best = top_pairs[0]
    best_mu = best[2]

    # island external edges
    island_set = {uid_to_i[u] for u in ISLAND_UIDS}
    external = {}
    for iu in ISLAND_UIDS:
        ii = uid_to_i[iu]
        ext = []
        for j in set(adj[ii] + radj[ii]):
            if j not in island_set:
                ext.append(node_uids[j])
        external[iu] = ext

    pair_reports = []
    for iu, pa, (d3, dxz, mu, pb, _ji) in top_pairs[:5]:
        pair_reports.append(
            stitch_pair_verdict(
                iu,
                mu,
                uid_to_i,
                coords,
                edges,
                edge_pairs,
                dirs,
                merge_sector_proxy(iu),
                merge_sector_proxy(mu),
            )
        )

    truck_to_start = stitch_pair_verdict(
        TRUCK_UID,
        ISLAND_UIDS[0],
        uid_to_i,
        coords,
        edges,
        edge_pairs,
        dirs,
        merge_sector_proxy(TRUCK_UID),
        merge_sector_proxy(ISLAND_UIDS[0]),
    )

    out = {
        "constants": {
            "BOUNDARY_STITCH_MAX_DIST_M": BOUNDARY_STITCH_MAX_DIST_M,
            "BOUNDARY_STITCH_Z_TOL_M": BOUNDARY_STITCH_Z_TOL_M,
            "BOUNDARY_STITCH_HEADING_DOT_MIN": BOUNDARY_STITCH_HEADING_DOT_MIN,
            "DEFAULT_CELL_SIZE": DEFAULT_CELL_SIZE,
            "candidate_generation": "per road_endpoint_uid: query_circle(center, 50m XZ); stats.candidates = len(road_endpoint_uids)",
        },
        "main_scc_size": len(main_scc),
        "island_size": len(ISLAND_UIDS),
        "island_external_edges": external,
        "top5_pairs_island_to_main_scc": pair_reports,
        "truck_to_island_start": truck_to_start,
        "minimal_pair": pair_reports[0],
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
