#!/usr/bin/env python3
"""City-to-SCC-Mapping: Welche Stadt liegt in welcher SCC,
und wuerde Stitch-Tuning tote Staedte anschliessen?

Output: JSON (Rohdaten) + Markdown-Report.
"""
from __future__ import annotations

import json
import math
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

TRUCK_UID = 3829402255816130719

# Stitch gates (wie spatial_match.rs)
RADIUS = 50.0
Z_TOL = 5.0
HEAD_DOT_MIN = 0.7


def read_cities_toml(path: str) -> list[dict]:
    """Parse test_cities.toml (hand-rolled, kein toml-dep)."""
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
            m = re.match(r'^(\w+)\s*=\s*(.+)$', line)
            if m:
                k, vraw = m.group(1), m.group(2).strip().strip('"')
                if k == "name":
                    cur["name"] = vraw
                elif k in ("x", "z"):
                    cur[k] = float(vraw)
    if cur and "name" in cur:
        cities.append(cur)
    return cities


def compute_tangents(v, adj_flat, adj_offsets, radj_flat, radj_offsets, coords):
    tangents = []
    for j in range(int(adj_offsets[v]), int(adj_offsets[v + 1])):
        w = int(adj_flat[j])
        d = coords[w] - coords[v]
        n = np.linalg.norm(d)
        if n > 0.001:
            tangents.append(d / n)
    for j in range(int(radj_offsets[v]), int(radj_offsets[v + 1])):
        u = int(radj_flat[j])
        d = coords[u] - coords[v]
        n = np.linalg.norm(d)
        if n > 0.001:
            tangents.append(d / n)
    return np.array(tangents, dtype=np.float64) if tangents else np.zeros((0, 3), dtype=np.float64)


def heading_dot(ta, tb):
    if len(ta) == 0 or len(tb) == 0:
        return -1.0
    return float((ta @ tb.T).max())


def uid_high32(uid):
    return uid >> 32


def main():
    t0 = time.time()

    # ---- Load graph ----
    print("Loading graph.json...", file=sys.stderr, flush=True)
    with open("graph.json", "rb") as f:
        g = json.load(f)
    nodes = g["nodes"]
    edges_list = g["edges"]
    N = len(nodes)
    print(f"  {N} nodes", file=sys.stderr, flush=True)

    uid_to_i = {}
    for i, n in enumerate(nodes):
        uid_to_i[int(n["uid"])] = i

    coords = np.zeros((N, 3), dtype=np.float64)
    for i, n in enumerate(nodes):
        coords[i] = (float(n["x"]), float(n["y"]), float(n["z"]))

    node_uids = np.array([int(n["uid"]) for n in nodes], dtype=np.int64)

    # ---- Flat adjacency ----
    print("Building adjacency...", file=sys.stderr, flush=True)
    adj_offsets = np.zeros(N + 1, dtype=np.int64)
    radj_offsets = np.zeros(N + 1, dtype=np.int64)
    has_edge = np.zeros(N, dtype=bool)

    ef_list, et_list = [], []
    for e in edges_list:
        fu = uid_to_i.get(int(e["from"]))
        tu = uid_to_i.get(int(e["to"]))
        if fu is None or tu is None:
            continue
        ef_list.append(fu)
        et_list.append(tu)
        adj_offsets[fu + 1] += 1
        radj_offsets[tu + 1] += 1
        has_edge[fu] = True
        has_edge[tu] = True

    E = len(ef_list)
    ef = np.array(ef_list, dtype=np.int64)
    et = np.array(et_list, dtype=np.int64)
    del ef_list, et_list, edges_list

    adj_offsets = np.cumsum(adj_offsets)
    radj_offsets = np.cumsum(radj_offsets)
    adj_flat = np.zeros(E, dtype=np.int64)
    radj_flat = np.zeros(E, dtype=np.int64)
    ap = adj_offsets[:-1].copy()
    rp = radj_offsets[:-1].copy()
    for i in range(E):
        adj_flat[ap[ef[i]]] = et[i]
        ap[ef[i]] += 1
        radj_flat[rp[et[i]]] = ef[i]
        rp[et[i]] += 1
    del ef, et, ap, rp

    # ---- Kosaraju ----
    print("Kosaraju...", file=sys.stderr, flush=True)
    visited = np.zeros(N, dtype=bool)
    order = np.zeros(N, dtype=np.int64)
    op = 0
    sv = np.zeros(N, dtype=np.int64)
    sp_arr = np.zeros(N, dtype=np.int64)

    for start in range(N):
        if visited[start]:
            continue
        sv[0] = start
        sp_arr[0] = int(adj_offsets[start])
        ptr = 1
        visited[start] = True
        while ptr > 0:
            v = int(sv[ptr - 1])
            pos = int(sp_arr[ptr - 1])
            end = int(adj_offsets[v + 1])
            if pos < end:
                w = int(adj_flat[pos])
                sp_arr[ptr - 1] = pos + 1
                if not visited[w]:
                    visited[w] = True
                    sv[ptr] = w
                    sp_arr[ptr] = int(adj_offsets[w])
                    ptr += 1
            else:
                ptr -= 1
                order[op] = v
                op += 1

    comp = np.full(N, -1, dtype=np.int64)
    comp_count = 0
    for idx in range(N - 1, -1, -1):
        v = int(order[idx])
        if comp[v] != -1:
            continue
        stack = [v]
        comp[v] = comp_count
        while stack:
            cv = stack.pop()
            for j in range(int(radj_offsets[cv]), int(radj_offsets[cv + 1])):
                w = int(radj_flat[j])
                if comp[w] == -1:
                    comp[w] = comp_count
                    stack.append(w)
        comp_count += 1
    print(f"  SCCs: {comp_count}", file=sys.stderr, flush=True)

    truck_i = uid_to_i[TRUCK_UID]
    main_scc = int(comp[truck_i])
    main_mask = comp == main_scc
    main_count = int(main_mask.sum())
    print(f"  Main SCC: {main_count} ({100*main_count/N:.1f}%)", file=sys.stderr, flush=True)

    comp_sizes = np.bincount(comp.astype(np.int64))

    # ---- Read cities ----
    cities = read_cities_toml("crates/map-parser/tests/fixtures/test_cities.toml")
    print(f"  Cities loaded: {len(cities)}", file=sys.stderr, flush=True)

    # ---- Build KD-tree for ROAD nodes only (city snap should use road nodes) ----
    road_mask = has_edge
    road_coords_xz = np.column_stack([coords[road_mask, 0], coords[road_mask, 2]])
    road_global_idx = np.where(road_mask)[0]
    tree_road_xz = cKDTree(road_coords_xz)

    # ---- Also build KD-tree for main SCC road nodes (for distance queries) ----
    main_road_mask = main_mask & has_edge
    main_road_xz = np.column_stack([coords[main_road_mask, 0], coords[main_road_mask, 2]])
    main_global_idx = np.where(main_road_mask)[0]
    tree_main_xz = cKDTree(main_road_xz)

    # ---- Map each city to nearest node + SCC ----
    city_rows = []
    for city in cities:
        name = city["name"]
        cx, cz = city["x"], city["z"]
        # Snap to nearest ROAD node
        dist_xz, idx = tree_road_xz.query([[cx, cz]], k=1)
        snap_dist = float(np.ravel(dist_xz)[0])
        ri = int(np.ravel(idx)[0])
        ni = int(road_global_idx[ri])
        node_uid = int(node_uids[ni])
        cid = int(comp[ni])
        scc_sz = int(comp_sizes[cid])
        in_main = cid == main_scc

        # Distance from this SCC to main SCC (nearest road endpoint pair)
        scc_dist_to_main = None
        scc_dy_to_main = None
        scc_dot_to_main = None

        if not in_main:
            # Compute distance from this SCC's nodes to main SCC
            scc_indices = np.where(comp == cid)[0]
            road_in_scc = scc_indices[has_edge[scc_indices]]
            if len(road_in_scc) > 0:
                q_coords = np.column_stack([coords[road_in_scc, 0], coords[road_in_scc, 2]])
                sub_dists, sub_idxs = tree_main_xz.query(q_coords, k=1)
                sub_dists_f = np.ravel(sub_dists)
                sub_idxs_f = np.ravel(sub_idxs)
                best_pos = int(np.argmin(sub_dists_f))
                best_road_idx = int(road_in_scc[best_pos])
                best_main_xz_idx = int(sub_idxs_f[best_pos])
                best_main_global = int(main_global_idx[best_main_xz_idx])

                # Full 3D distance
                d = coords[best_main_global] - coords[best_road_idx]
                scc_dist_to_main = float(np.linalg.norm(d))
                scc_dy_to_main = abs(float(d[1]))
                scc_dot_to_main = -999.0

                # Tangents for best pair
                ta = compute_tangents(best_road_idx, adj_flat, adj_offsets,
                                      radj_flat, radj_offsets, coords)
                tb = compute_tangents(best_main_global, adj_flat, adj_offsets,
                                      radj_flat, radj_offsets, coords)
                scc_dot_to_main = heading_dot(ta, tb)
            else:
                scc_dist_to_main = 999999.0
                scc_dy_to_main = 999.0
                scc_dot_to_main = -1.0

        city_rows.append({
            "name": name,
            "city_x": cx,
            "city_z": cz,
            "snap_dist_m": round(snap_dist, 1),
            "snap_node_uid": node_uid,
            "component_id": cid,
            "component_size": scc_sz,
            "in_main_scc": in_main,
            "scc_dist_to_main_m": round(scc_dist_to_main, 1) if scc_dist_to_main is not None else None,
            "scc_dy_to_main_m": round(scc_dy_to_main, 1) if scc_dy_to_main is not None else None,
            "scc_dot_to_main": round(scc_dot_to_main, 3) if scc_dot_to_main is not None else None,
        })

    # ---- Determine which cities would dock per scenario ----
    dead = [r for r in city_rows if not r["in_main_scc"]]
    alive = [r for r in city_rows if r["in_main_scc"]]

    scenarios = [
        ("radius50_z5_h07", 50.0, 5.0, 0.7),
        ("radius100_z5_h07", 100.0, 5.0, 0.7),
        ("radius150_z5_h07", 150.0, 5.0, 0.7),
        ("radius100_z10_h06", 100.0, 10.0, 0.6),
        ("radius150_z10_h06", 150.0, 10.0, 0.6),
    ]

    scenario_results = {}
    for sname, sradius, sztol, shdot in scenarios:
        docked = []
        for r in dead:
            if r["scc_dist_to_main_m"] is None:
                continue
            d = r["scc_dist_to_main_m"]
            dy = r["scc_dy_to_main_m"]
            dot = r["scc_dot_to_main"]
            # same-sector check via uid_high32
            snap_node_uid = r["snap_node_uid"]
            # We need the main-node uid for the same-sector check
            # But we didn't store it. Use uid_high32 of the city's snap node
            # and... we don't have the main-side uid without looking it up.
            # The best pair's main uid is complex. Let's look it up.

            # Actually, we already computed the best pair but didn't store the UIDs.
            # For the scenario check, same_sector is checked via uid_high32 of
            # the pair. We need to determine it.

            # For now: approximate. We'll recompute for dead cities.
            passes_sector = True  # assume cross-sector initially, verify below

            # Check gate
            if d <= sradius and dy <= sztol and dot >= shdot:
                docked.append(r["name"])
        scenario_results[sname] = docked

    # Recompute same-sector for dead cities' best pairs
    # (We need the main-side uid for each dead city's SCC best pair)
    print("Computing same-sector for dead cities...", file=sys.stderr, flush=True)
    dead_scc_ids = set(r["component_id"] for r in dead if r["scc_dist_to_main_m"] is not None)
    scc_best_main_uid = {}
    for cid in dead_scc_ids:
        scc_indices = np.where(comp == cid)[0]
        road_in_scc = scc_indices[has_edge[scc_indices]]
        if len(road_in_scc) == 0:
            continue
        q_coords = np.column_stack([coords[road_in_scc, 0], coords[road_in_scc, 2]])
        sub_dists, sub_idxs = tree_main_xz.query(q_coords, k=1)
        sub_dists_f = np.ravel(sub_dists)
        sub_idxs_f = np.ravel(sub_idxs)
        best_pos = int(np.argmin(sub_dists_f))
        best_main_idx_in_main_road = int(sub_idxs_f[best_pos])
        best_main_global = int(main_global_idx[best_main_idx_in_main_road])
        scc_best_main_uid[cid] = int(node_uids[best_main_global])

    # Re-do scenario check with same-sector info
    scenario_results2 = {}
    for sname, sradius, sztol, shdot in scenarios:
        docked_cities = []
        docked_sccs = set()
        docked_nodes = 0
        for r in dead:
            if r["scc_dist_to_main_m"] is None:
                continue
            cid = r["component_id"]
            d = r["scc_dist_to_main_m"]
            dy = r["scc_dy_to_main_m"]
            dot = r["scc_dot_to_main"]
            main_uid = scc_best_main_uid.get(cid, 0)
            island_uid = r["snap_node_uid"]
            same_sector = uid_high32(int(main_uid)) == uid_high32(int(island_uid))

            if d <= sradius and dy <= sztol and dot >= shdot and not same_sector and dot >= -0.5:
                docked_cities.append(r["name"])
                if cid not in docked_sccs:
                    docked_sccs.add(cid)
                    docked_nodes += r["component_size"]
        scenario_results2[sname] = {
            "cities": sorted(docked_cities),
            "city_count": len(docked_cities),
            "scc_count": len(docked_sccs),
            "nodes_gained": docked_nodes,
        }

    # ---- Additional: how many cities in main SCC? ----
    cities_in_main = [r for r in city_rows if r["in_main_scc"]]

    # ---- Dead city detail ----
    dead_detail = []
    for r in dead:
        cid = r["component_id"]
        main_uid = scc_best_main_uid.get(cid, 0)
        island_uid = r["snap_node_uid"]
        same_sec = uid_high32(int(main_uid)) == uid_high32(int(island_uid))
        dead_detail.append({
            "name": r["name"],
            "component_size": r["component_size"],
            "scc_dist_to_main_m": r["scc_dist_to_main_m"],
            "scc_dy_to_main_m": r["scc_dy_to_main_m"],
            "scc_dot_to_main": r["scc_dot_to_main"],
            "same_sector": same_sec,
            "snap_dist_m": r["snap_dist_m"],
            "city_x": r["city_x"],
            "city_z": r["city_z"],
        })

    # Distance bucket for each dead city
    for d in dead_detail:
        dist = d["scc_dist_to_main_m"]
        if dist is None:
            d["distance_bucket"] = "NO_SNAP"
        elif dist < 50:
            d["distance_bucket"] = "<50m"
        elif dist < 90:
            d["distance_bucket"] = "50-90m"
        elif dist < 150:
            d["distance_bucket"] = "90-150m"
        elif dist < 300:
            d["distance_bucket"] = "150-300m"
        elif dist < 1000:
            d["distance_bucket"] = "300-1000m"
        else:
            d["distance_bucket"] = ">1000m"

    # ---- Per-bucket city counts ----
    bucket_counts = defaultdict(int)
    bucket_cities = defaultdict(list)
    for d in dead_detail:
        bk = d["distance_bucket"]
        bucket_counts[bk] += 1
        bucket_cities[bk].append(d["name"])

    # ---- City pairs that would become routable ----
    alive_names = set(r["name"] for r in alive)
    total_pairs = len(alive_names) * len(alive_names)  # main + main

    # For scenario s2: new_pairs = (alive + docked) * (alive + docked) - alive * alive
    for sname, sdata in scenario_results2.items():
        docked_set = set(sdata["cities"])
        new_pairs = ((len(alive_names) + len(docked_set)) ** 2) - (len(alive_names) ** 2)
        sdata["routable_city_pairs"] = new_pairs
        # Estimate routing rate: each pair is 1/90 of route pairs (from 20/90 current)
        current_pairs = len(alive_names) * (len(alive_names) - 1)  # 18*17 = 306? Wait...
        # Actually the metric is "20 von 90 Stadt-Paaren = 22%". 90 = 10*9 (10 cities tested).
        # With 40 cities, theoretical max = 40*39 = 1560 pairs.
        # Let's use: route_rate ~ cities_in_main / total_cities (simplified)
        sdata["estimated_route_rate_pct"] = round(
            (len(alive_names) + len(docked_set)) / len(cities) * 100, 1
        )

    # ---- Output ----
    t1 = time.time()
    out = {
        "meta": {
            "elapsed_s": round(t1 - t0, 1),
            "graph_nodes": N,
            "main_scc_size": main_count,
            "main_scc_pct": round(100 * main_count / N, 1),
        },
        "city_summary": {
            "total_cities": len(cities),
            "cities_in_main_scc": len(cities_in_main),
            "cities_in_main_names": sorted([r["name"] for r in cities_in_main]),
            "dead_cities_count": len(dead),
        },
        "dead_city_detail": dead_detail,
        "dead_cities_by_distance": {k: {"count": v, "cities": bucket_cities[k]}
                                     for k, v in sorted(bucket_counts.items())},
        "scenario_results": scenario_results2,
        "alive_cities": sorted([r["name"] for r in cities_in_main]),
    }

    out_path = Path("outputs/2026-06-14/graph-audit-tmp/city_scc_data.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
