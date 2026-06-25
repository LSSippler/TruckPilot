#!/usr/bin/env python3
"""Gap-Pattern-Messung: Vermisst ALLE Nicht-Haupt-SCCs gegen die Haupt-SCC.

Output: JSON mit aggregierten Statistiken (Distanzen, Gate-Ergebnisse, Szenarien).
"""
from __future__ import annotations

import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

# ---- Stitch-Gate-Konstanten (spatial_match.rs) ----
BOUNDARY_STITCH_MAX_DIST_M = 50.0
BOUNDARY_STITCH_Z_TOL_M = 5.0
BOUNDARY_STITCH_HEADING_DOT_MIN = 0.7
VIRTUAL_SECTOR_SIZE = 4096.0

TRUCK_UID = 3829402255816130719


def uid_high32(uid: int) -> int:
    return uid >> 32


def compute_tangents(v, adj_flat, adj_offsets, radj_flat, radj_offsets, coords):
    """Berechne Einheits-Tangentenvektoren fuer Node v.
    Spiegelt island_gap_forensics.py::road_tangents_from_graph.
    """
    tangents = []
    for j in range(adj_offsets[v], adj_offsets[v + 1]):
        w = adj_flat[j]
        delta = coords[w] - coords[v]
        n = np.linalg.norm(delta)
        if n > 0.001:
            tangents.append(delta / n)
    for j in range(radj_offsets[v], radj_offsets[v + 1]):
        u = radj_flat[j]
        delta = coords[u] - coords[v]
        n = np.linalg.norm(delta)
        if n > 0.001:
            tangents.append(delta / n)
    if not tangents:
        return np.zeros((0, 3), dtype=np.float64)
    return np.array(tangents, dtype=np.float64)


def heading_dot(ta, tb):
    """Max dot product zwischen zwei Tangenten-Mengen."""
    if len(ta) == 0 or len(tb) == 0:
        return -1.0
    dots = ta @ tb.T
    return float(dots.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    ap.add_argument("--out-json", default="outputs/2026-06-14/graph-audit-tmp/gap_pattern_data.json")
    args = ap.parse_args()

    t0 = time.time()

    # ---- 1. Load ----
    print("Loading graph...", file=sys.stderr)
    with open(Path(args.graph), "rb") as f:
        g = json.load(f)
    nodes = g["nodes"]
    edges_list = g["edges"]
    N = len(nodes)
    E = len(edges_list)
    print(f"  {N} nodes, {E} edges", file=sys.stderr)

    uid_to_i = {}
    for i, n in enumerate(nodes):
        uid_to_i[int(n["uid"])] = i

    coords = np.zeros((N, 3), dtype=np.float64)
    for i, n in enumerate(nodes):
        coords[i] = (float(n["x"]), float(n["y"]), float(n["z"]))

    node_uids = np.array([int(n["uid"]) for n in nodes], dtype=np.int64)

    # ---- 2. Flat adjacency ----
    print("Building adjacency...", file=sys.stderr)
    adj_offsets = np.zeros(N + 1, dtype=np.int64)
    radj_offsets = np.zeros(N + 1, dtype=np.int64)
    has_edge = np.zeros(N, dtype=bool)

    edge_froms = []
    edge_tos = []
    for e in edges_list:
        fu = uid_to_i.get(int(e["from"]))
        tu = uid_to_i.get(int(e["to"]))
        if fu is None or tu is None:
            continue
        edge_froms.append(fu)
        edge_tos.append(tu)
        adj_offsets[fu + 1] += 1
        radj_offsets[tu + 1] += 1
        has_edge[fu] = True
        has_edge[tu] = True

    E_valid = len(edge_froms)
    ef = np.array(edge_froms, dtype=np.int64)
    et = np.array(edge_tos, dtype=np.int64)
    del edge_froms, edge_tos, edges_list

    adj_offsets = np.cumsum(adj_offsets)
    radj_offsets = np.cumsum(radj_offsets)

    adj_flat = np.zeros(E_valid, dtype=np.int64)
    radj_flat = np.zeros(E_valid, dtype=np.int64)
    ap_pos = adj_offsets[:-1].copy()
    rp_pos = radj_offsets[:-1].copy()
    for i in range(E_valid):
        fu = ef[i]
        tu = et[i]
        adj_flat[ap_pos[fu]] = tu
        ap_pos[fu] += 1
        radj_flat[rp_pos[tu]] = fu
        rp_pos[tu] += 1
    del ef, et, ap_pos, rp_pos

    road_node_count = int(has_edge.sum())
    print(f"  {E_valid} valid edges, {road_node_count} nodes with edges", file=sys.stderr)

    # ---- 3. Kosaraju (iterativ, low-memory stack) ----
    print("Kosaraju pass 1 (forward DFS)...", file=sys.stderr)
    visited = np.zeros(N, dtype=bool)
    order = np.zeros(N, dtype=np.int64)
    order_ptr = 0

    stack_v = np.zeros(N, dtype=np.int64)
    stack_pos = np.zeros(N, dtype=np.int64)

    for start in range(N):
        if visited[start]:
            continue
        sp = 1
        stack_v[0] = start
        stack_pos[0] = int(adj_offsets[start])
        visited[start] = True
        while sp > 0:
            v = int(stack_v[sp - 1])
            pos = int(stack_pos[sp - 1])
            end = int(adj_offsets[v + 1])
            if pos < end:
                w = int(adj_flat[pos])
                stack_pos[sp - 1] = pos + 1
                if not visited[w]:
                    visited[w] = True
                    stack_v[sp] = w
                    stack_pos[sp] = int(adj_offsets[w])
                    sp += 1
            else:
                sp -= 1
                order[order_ptr] = v
                order_ptr += 1

    print("Kosaraju pass 2 (reverse DFS)...", file=sys.stderr)
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

    print(f"  SCC count: {comp_count}", file=sys.stderr)

    # ---- 4. Main SCC ----
    truck_i = uid_to_i[TRUCK_UID]
    main_scc = int(comp[truck_i])
    main_mask = comp == main_scc
    main_count = int(main_mask.sum())
    print(f"  Main SCC size: {main_count}  ({100*main_count/N:.1f}%)", file=sys.stderr)

    # ---- 5. SCC size distribution ----
    comp_sizes = np.bincount(comp.astype(np.int64))  # comp_count values

    size_dist = defaultdict(int)
    size_dist_roads = defaultdict(int)
    for cid in range(comp_count):
        s = int(comp_sizes[cid])
        if s == 1:
            size_dist["1"] += 1
        elif s <= 10:
            size_dist["2-10"] += 1
        elif s <= 100:
            size_dist["11-100"] += 1
        elif s <= 1000:
            size_dist["101-1000"] += 1
        else:
            size_dist[">1000"] += 1

    # Count road-endpoint SCCs per size bucket
    for i in range(N):
        if has_edge[i] and comp[i] != main_scc:
            cid = int(comp[i])
            s = int(comp_sizes[cid])
            if s == 1:
                size_dist_roads["1"] += 0  # count per component, not per node
            # We'll count per-component later

    # ---- 6. Nearest-main distance per non-main SCC ----
    print("Building KD-tree of main SCC...", file=sys.stderr)
    main_global_indices = np.where(main_mask)[0]
    main_tree = cKDTree(coords[main_mask])

    print("Querying nearest main-SCC neighbor per non-main road endpoint...", file=sys.stderr)
    non_main_road_mask = (~main_mask) & has_edge
    non_main_road_idx = np.where(non_main_road_mask)[0]
    nm_count = len(non_main_road_idx)
    print(f"  {nm_count} non-main road endpoints to query", file=sys.stderr)

    # Per-SCC best values (arrays indexed by comp_id)
    n_comp = comp_count
    best_dist_arr = np.full(n_comp, np.inf, dtype=np.float64)
    best_dy_arr = np.zeros(n_comp, dtype=np.float64)
    best_dot_arr = np.full(n_comp, -1.0, dtype=np.float64)
    best_same_sector = np.zeros(n_comp, dtype=bool)
    best_main_global = np.full(n_comp, -1, dtype=np.int64)
    best_island_global = np.full(n_comp, -1, dtype=np.int64)
    best_island_uid_arr = np.zeros(n_comp, dtype=np.int64)
    best_main_uid_arr = np.zeros(n_comp, dtype=np.int64)

    # Batch query
    query_coords = coords[non_main_road_idx]
    batch_dists, batch_idxs = main_tree.query(query_coords, k=1)
    del query_coords

    for pos in range(nm_count):
        i = int(non_main_road_idx[pos])
        cid = int(comp[i])
        if cid == main_scc:
            continue
        d = float(batch_dists[pos])
        if d >= best_dist_arr[cid]:
            continue
        best_dist_arr[cid] = d
        best_island_global[cid] = i
        best_main_global[cid] = int(main_global_indices[int(batch_idxs[pos])])
        best_island_uid_arr[cid] = int(node_uids[i])
        best_main_uid_arr[cid] = int(node_uids[int(main_global_indices[int(batch_idxs[pos])])])

    del batch_dists, batch_idxs

    # ---- 7. Compute dy, heading_dot, same_sector per SCC ----
    print("Computing gate values for best pairs...", file=sys.stderr)
    sccs_analyzed = 0
    for cid in range(n_comp):
        if cid == main_scc:
            continue
        if best_island_global[cid] == -1:
            continue  # no road endpoints
        ii = int(best_island_global[cid])
        mi = int(best_main_global[cid])
        p_island = coords[ii]
        p_main = coords[mi]
        dx, dy, dz = p_main - p_island
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        best_dist_arr[cid] = d3
        best_dy_arr[cid] = abs(float(dy))

        # heading dot
        ta = compute_tangents(ii, adj_flat, adj_offsets, radj_flat, radj_offsets, coords)
        tb = compute_tangents(mi, adj_flat, adj_offsets, radj_flat, radj_offsets, coords)
        best_dot_arr[cid] = heading_dot(ta, tb)

        # same sector
        best_same_sector[cid] = uid_high32(int(node_uids[ii])) == uid_high32(int(node_uids[mi]))

        sccs_analyzed += 1

    print(f"  Analyzed {sccs_analyzed} non-main SCCs with road endpoints", file=sys.stderr)

    # ---- 8. Count road-endpoint SCCs per size bucket ----
    road_scc_by_size = defaultdict(int)
    road_scc_nodes_by_size = defaultdict(int)
    for cid in range(n_comp):
        if cid == main_scc:
            continue
        if best_island_global[cid] == -1:
            continue
        s = int(comp_sizes[cid])
        road_scc_nodes_by_size[f"size={s}"] += s
        if s == 1:
            road_scc_by_size["1"] += 1
        elif s <= 10:
            road_scc_by_size["2-10"] += 1
        elif s <= 100:
            road_scc_by_size["11-100"] += 1
        elif s <= 1000:
            road_scc_by_size["101-1000"] += 1
        else:
            road_scc_by_size[">1000"] += 1

    # ---- 9. Aggregate statistics ----
    # 9a. Distance histogram
    dist_buckets = {"<50m": 0, "50-90m": 0, "90-150m": 0, "150-300m": 0, "300-1000m": 0, ">1000m": 0}
    dist_node_counts = defaultdict(int)
    for cid in range(n_comp):
        if best_island_global[cid] == -1:
            continue
        d = best_dist_arr[cid]
        s = int(comp_sizes[cid])
        if d < 50:
            dist_buckets["<50m"] += 1
            dist_node_counts["<50m"] += s
        elif d < 90:
            dist_buckets["50-90m"] += 1
            dist_node_counts["50-90m"] += s
        elif d < 150:
            dist_buckets["90-150m"] += 1
            dist_node_counts["90-150m"] += s
        elif d < 300:
            dist_buckets["150-300m"] += 1
            dist_node_counts["150-300m"] += s
        elif d < 1000:
            dist_buckets["300-1000m"] += 1
            dist_node_counts["300-1000m"] += s
        else:
            dist_buckets[">1000m"] += 1
            dist_node_counts[">1000m"] += s

    # 9b. Gate analysis
    gate_results = {
        "dy_gt_5m": 0,
        "heading_below_07": 0,
        "same_sector": 0,
        "no_tangents": 0,
    }

    pass_scenarios = {
        "radius50_z5_head07_cross": 0,
        "radius100_z5_head07_cross": 0,
        "radius150_z5_head07_cross": 0,
        "radius100_z10_head06_cross": 0,
        "radius150_z10_head06_cross": 0,
    }

    pass_nodes = {k: 0 for k in pass_scenarios}

    # Cross-table: for each distance bucket, count failures per gate
    cross_dist = {
        "<50m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
        "50-90m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
        "90-150m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
        "150-300m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
        "300-1000m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
        ">1000m": {"total": 0, "z": 0, "heading": 0, "same_sector": 0, "no_tangents": 0, "pass_all_50": 0},
    }

    dy_fail_count = 0
    head_fail_count = 0
    sector_fail_count = 0

    for cid in range(n_comp):
        if best_island_global[cid] == -1:
            continue
        d = best_dist_arr[cid]
        dy = best_dy_arr[cid]
        dot = best_dot_arr[cid]
        sec = best_same_sector[cid]
        s = int(comp_sizes[cid])

        # Bucket
        if d < 50:
            bk = "<50m"
        elif d < 90:
            bk = "50-90m"
        elif d < 150:
            bk = "90-150m"
        elif d < 300:
            bk = "150-300m"
        elif d < 1000:
            bk = "300-1000m"
        else:
            bk = ">1000m"

        cross_dist[bk]["total"] += 1
        fail_z = dy > BOUNDARY_STITCH_Z_TOL_M
        fail_head = dot < BOUNDARY_STITCH_HEADING_DOT_MIN
        fail_sector = sec
        fail_no_tang = dot < -0.5

        if fail_z:
            cross_dist[bk]["z"] += 1
        if fail_head:
            cross_dist[bk]["heading"] += 1
        if fail_sector:
            cross_dist[bk]["same_sector"] += 1
        if fail_no_tang:
            cross_dist[bk]["no_tangents"] += 1

        # Check pass with current gates (dist <= 50)
        if d <= BOUNDARY_STITCH_MAX_DIST_M:
            if not fail_z and not fail_head and not fail_sector and not fail_no_tang:
                cross_dist[bk]["pass_all_50"] += 1
                pass_scenarios["radius50_z5_head07_cross"] += 1
                pass_nodes["radius50_z5_head07_cross"] += s

        # Scenarios
        for radius, ztol, hdot, key in [
            (100, 5.0, 0.7, "radius100_z5_head07_cross"),
            (150, 5.0, 0.7, "radius150_z5_head07_cross"),
            (100, 10.0, 0.6, "radius100_z10_head06_cross"),
            (150, 10.0, 0.6, "radius150_z10_head06_cross"),
        ]:
            if d <= radius and dy <= ztol and dot >= hdot and not sec and dot >= -0.5:
                pass_scenarios[key] += 1
                pass_nodes[key] += s

    # 9c. Gate failure summary (all SCCs)
    for cid in range(n_comp):
        if best_island_global[cid] == -1:
            continue
        if best_dy_arr[cid] > BOUNDARY_STITCH_Z_TOL_M:
            gate_results["dy_gt_5m"] += 1
        if best_dot_arr[cid] < BOUNDARY_STITCH_HEADING_DOT_MIN:
            gate_results["heading_below_07"] += 1
        if best_same_sector[cid]:
            gate_results["same_sector"] += 1
        if best_dot_arr[cid] < -0.5:
            gate_results["no_tangents"] += 1

    # ---- 10. Detailed distance breakdown ----
    dists_all = [best_dist_arr[cid] for cid in range(n_comp) if best_island_global[cid] != -1]
    dys_all = [best_dy_arr[cid] for cid in range(n_comp) if best_island_global[cid] != -1]
    dots_all = [best_dot_arr[cid] for cid in range(n_comp) if best_island_global[cid] != -1]

    # Percentile info
    dists_sorted = sorted(dists_all)
    n_dists = len(dists_sorted)
    p25 = dists_sorted[int(n_dists * 0.25)]
    p50 = dists_sorted[int(n_dists * 0.50)]
    p75 = dists_sorted[int(n_dists * 0.75)]
    p90 = dists_sorted[int(n_dists * 0.90)]

    # ---- 11. Output ----
    t1 = time.time()
    elapsed = t1 - t0

    out = {
        "meta": {"elapsed_s": round(elapsed, 1), "graph_nodes": N, "graph_edges": E_valid,
                 "main_scc_size": main_count, "main_scc_pct": round(100 * main_count / N, 1),
                 "scc_count": comp_count,
                 "non_main_scc_with_roads": sccs_analyzed},
        "size_distribution_all_sccs": dict(size_dist),
        "size_distribution_road_sccs": dict(road_scc_by_size),
        "road_scc_nodes_by_size": dict(road_scc_nodes_by_size),
        "distance_histogram_sccs": dist_buckets,
        "distance_histogram_nodes": dict(dist_node_counts),
        "distance_percentiles_m": {"p25": round(p25, 1), "p50": round(p50, 1),
                                   "p75": round(p75, 1), "p90": round(p90, 1)},
        "distance_min": round(min(dists_all), 1) if dists_all else None,
        "distance_max": round(max(dists_all), 1) if dists_all else None,
        "gate_failures": gate_results,
        "gate_cross_table_by_distance": cross_dist,
        "scenarios": pass_scenarios,
        "scenario_nodes": pass_nodes,
        "dist_all_sample": [round(d, 1) for d in dists_sorted[:50]],
        "dy_all_sample": [round(d, 1) for d in dys_all[:50]],
        "dot_all_sample": [round(d, 2) for d in dots_all[:50]],
    }

    out_path = Path(args.out_json)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    import argparse
    main()
