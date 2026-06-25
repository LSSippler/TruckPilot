#!/usr/bin/env python3
"""Empirical NavCurve vs Road lane-offset geometry at prefab->road transitions."""
from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path

GRAPH = Path(__file__).resolve().parents[1] / "graph.json"
OUT_JSON = Path(__file__).resolve().parents[1] / "outputs/2026-06-14/routing-audit-tmp/navcurve_vs_road.json"
TARGET_LANE_FROM_RIGHT = 0


def load_graph():
    with GRAPH.open(encoding="utf-8") as f:
        return json.load(f)


def heading_deg(dx: float, dz: float) -> float:
    return math.degrees(math.atan2(dx, -dz)) % 360.0


def right_normal(hdg_deg: float) -> tuple[float, float]:
    r = math.radians(hdg_deg)
    return math.cos(r), math.sin(r)


def offset_a(lanes: int, w: float, ro: float) -> float:
    tl = lanes - 1 - TARGET_LANE_FROM_RIGHT
    return (tl + 0.5) * w + ro


def offset_b(lanes: int, w: float, ro: float) -> float:
    tl = lanes - 1 - TARGET_LANE_FROM_RIGHT
    return (tl + 0.5 - lanes / 2.0) * w + ro


def dist2d(ax, az, bx, bz) -> float:
    return math.hypot(ax - bx, az - bz)


def angle_diff_deg(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def signed_lateral(hdg_deg: float, cx: float, cz: float, px: float, pz: float) -> float:
    """Match lane-follower: positive = point right of center along heading."""
    r = math.radians(hdg_deg)
    fwd_x = math.sin(r)
    fwd_z = -math.cos(r)
    return fwd_x * (pz - cz) - fwd_z * (px - cx)


def main():
    g = load_graph()
    nodes = {n["uid"]: n for n in g["nodes"]}

    # Road edges by start node (outgoing) and end node (incoming)
    road_out: dict[int, list[dict]] = defaultdict(list)
    road_in: dict[int, list[dict]] = defaultdict(list)
    for e in g["edges"]:
        d = e.get("direction", "")
        if d not in ("forward", "backward", "bidirectional_unknown"):
            continue
        lanes = int(e.get("lanes", 0))
        if lanes < 2:
            continue
        road_out[e["from"]].append(e)
        road_in[e["to"]].append(e)

    # Prefab paths indexed by exit/entry node
    prefab_exit: dict[int, list[dict]] = defaultdict(list)  # to_node
    prefab_entry: dict[int, list[dict]] = defaultdict(list)  # from_node
    for p in g["prefab_ai_paths"]:
        pts = p.get("spline_points") or []
        if len(pts) < 2:
            continue
        prefab_exit[p["to_node_uid"]].append(p)
        prefab_entry[p["from_node_uid"]].append(p)

    samples = []

    def analyze(kind: str, node_uid: int, path: dict, edge: dict, nav_pt, road_hdg, prefab_hdg):
        n = nodes.get(node_uid)
        if not n:
            return None
        cx, cz = float(n["x"]), float(n["z"])
        nx, nz = float(nav_pt[0]), float(nav_pt[2])
        lanes = max(int(edge.get("lanes", 1)), 1)
        opp = int(edge.get("lanes_opposite", 0))
        w = float(edge.get("lane_width_m", 3.75))
        ro = float(edge.get("road_offset_m", 0.0))
        oa = offset_a(lanes, w, ro)
        ob = offset_b(lanes, w, ro)
        nx_n, nz_n = right_normal(road_hdg)
        tx_a = cx + nx_n * oa
        tz_a = cz + nz_n * oa
        tx_b = cx + nx_n * ob
        tz_b = cz + nz_n * ob
        nav_lat = signed_lateral(road_hdg, cx, cz, nx, nz)
        d_nav_center = dist2d(nx, nz, cx, cz)
        d_nav_target_a = dist2d(nx, nz, tx_a, tz_a)
        d_nav_target_b = dist2d(nx, nz, tx_b, tz_b)
        other_uid = edge["to"] if kind == "prefab_to_road" else edge["from"]
        o = nodes.get(other_uid)
        if not o:
            return None
        edge_len = dist2d(cx, cz, float(o["x"]), float(o["z"]))
        return {
            "kind": kind,
            "node_uid": node_uid,
            "edge_uid": edge.get("uid"),
            "path_from": path["from_node_uid"],
            "path_to": path["to_node_uid"],
            "lanes": lanes,
            "lanes_opposite": opp,
            "lane_width_m": w,
            "road_offset_m": ro,
            "offset_a": round(oa, 3),
            "offset_b": round(ob, 3),
            "nav_lat_from_center": round(nav_lat, 3),
            "dist_nav_center": round(d_nav_center, 3),
            "dist_nav_road_target_a": round(d_nav_target_a, 3),
            "dist_nav_road_target_b": round(d_nav_target_b, 3),
            "err_a": round(abs(nav_lat - oa), 3),
            "err_b": round(abs(nav_lat - ob), 3),
            "road_hdg_deg": round(road_hdg, 2),
            "prefab_hdg_deg": round(prefab_hdg, 2),
            "hdg_diff_deg": round(angle_diff_deg(road_hdg, prefab_hdg), 2),
            "edge_len_m": round(edge_len, 1),
            "nav_to_node_m": round(dist2d(nx, nz, cx, cz), 3),
            "start_lane_idx": path.get("start_lane_idx"),
            "end_lane_idx": path.get("end_lane_idx"),
        }

    # Prefab -> Road at exit node
    for node_uid, paths in prefab_exit.items():
        edges = road_out.get(node_uid)
        if not edges:
            continue
        for path in paths:
            pts = path["spline_points"]
            nav = pts[-1]
            if len(pts) >= 2:
                p0, p1 = pts[-2], pts[-1]
                prefab_hdg = heading_deg(p1[0] - p0[0], p1[2] - p0[2])
            else:
                prefab_hdg = 0.0
            for edge in edges:
                o = nodes.get(edge["to"])
                if not o:
                    continue
                road_hdg = heading_deg(float(o["x"]) - nodes[node_uid]["x"], float(o["z"]) - nodes[node_uid]["z"])
                if angle_diff_deg(road_hdg, prefab_hdg) > 25:
                    continue
                s = analyze("prefab_to_road", node_uid, path, edge, nav, road_hdg, prefab_hdg)
                if s and s["edge_len_m"] >= 30 and s["nav_to_node_m"] <= 8:
                    samples.append(s)

    # Road -> Prefab at entry node
    for node_uid, paths in prefab_entry.items():
        edges = road_in.get(node_uid)
        if not edges:
            continue
        for path in paths:
            pts = path["spline_points"]
            nav = pts[0]
            if len(pts) >= 2:
                p0, p1 = pts[0], pts[1]
                prefab_hdg = heading_deg(p1[0] - p0[0], p1[2] - p0[2])
            else:
                prefab_hdg = 0.0
            for edge in edges:
                o = nodes.get(edge["from"])
                if not o:
                    continue
                road_hdg = heading_deg(nodes[node_uid]["x"] - float(o["x"]), nodes[node_uid]["z"] - float(o["z"]))
                if angle_diff_deg(road_hdg, prefab_hdg) > 25:
                    continue
                s = analyze("road_to_prefab", node_uid, path, edge, nav, road_hdg, prefab_hdg)
                if s and s["edge_len_m"] >= 30 and s["nav_to_node_m"] <= 8:
                    samples.append(s)

    # Sort by best geometry (low heading diff, long edge)
    samples.sort(key=lambda s: (s["hdg_diff_deg"], -s["edge_len_m"]))

    # Deduplicate by (node, edge) keep best prefab path
    best: dict[tuple, dict] = {}
    for s in samples:
        key = (s["node_uid"], s["edge_uid"], s["kind"])
        if key not in best or s["err_b"] < best[key]["err_b"]:
            best[key] = s
    uniq = list(best.values())
    uniq.sort(key=lambda s: s["err_a"] - s["err_b"])

    # Pick diverse report set: 10 with lowest err_b, 10 with lowest err_a, mix road_offset
    report = []
    seen_ro = set()
    for s in sorted(uniq, key=lambda x: x["dist_nav_road_target_a"], reverse=True)[:5]:
        report.append(s)
    for s in sorted(uniq, key=lambda x: x["err_b"])[:8]:
        if s not in report:
            report.append(s)
    for s in sorted(uniq, key=lambda x: x["err_a"])[:5]:
        if s not in report and len(report) < 12:
            report.append(s)
    # ensure road_offset=0 and >0
    for ro_filter in [0.0, None]:
        for s in uniq:
            if ro_filter == 0.0 and abs(s["road_offset_m"]) > 1e-6:
                continue
            if ro_filter is None and abs(s["road_offset_m"]) < 1e-6:
                continue
            if s not in report and len(report) < 15:
                report.append(s)

    # Stats over all uniq
    def med(vals):
        v = sorted(vals)
        return v[len(v) // 2] if v else 0

    stats_all = {
        "n_transitions": len(uniq),
        "median_dist_nav_target_a": med([s["dist_nav_road_target_a"] for s in uniq]),
        "median_dist_nav_target_b": med([s["dist_nav_road_target_b"] for s in uniq]),
        "median_err_a": med([s["err_a"] for s in uniq]),
        "median_err_b": med([s["err_b"] for s in uniq]),
        "pct_err_a_lt_1p5": sum(1 for s in uniq if s["err_a"] < 1.5) / max(len(uniq), 1) * 100,
        "pct_err_b_lt_1p5": sum(1 for s in uniq if s["err_b"] < 1.5) / max(len(uniq), 1) * 100,
    }

    ro0 = [s for s in uniq if abs(s["road_offset_m"]) < 1e-6]
    ro_nz = [s for s in uniq if abs(s["road_offset_m"]) >= 1e-6]
    stats_ro0 = {
        "n": len(ro0),
        "median_err_a": med([s["err_a"] for s in ro0]),
        "median_err_b": med([s["err_b"] for s in ro0]),
        "median_dist_a": med([s["dist_nav_road_target_a"] for s in ro0]),
        "median_dist_b": med([s["dist_nav_road_target_b"] for s in ro0]),
    }
    stats_ro_nz = {
        "n": len(ro_nz),
        "median_err_a": med([s["err_a"] for s in ro_nz]),
        "median_err_b": med([s["err_b"] for s in ro_nz]),
        "median_dist_a": med([s["dist_nav_road_target_a"] for s in ro_nz]),
        "median_dist_b": med([s["dist_nav_road_target_b"] for s in ro_nz]),
    }

    # Same-sign: nav curve and target on same side of center (positive = right)
    same_sign = [s for s in uniq if s["nav_lat_from_center"] * s["offset_b"] > 0]
    stats_same = {
        "n": len(same_sign),
        "median_err_a": med([s["err_a"] for s in same_sign]),
        "median_err_b": med([s["err_b"] for s in same_sign]),
        "pct_err_b_lt_1p5": sum(1 for s in same_sign if s["err_b"] < 1.5) / max(len(same_sign), 1) * 100,
    }

    lanes3_ro0 = [s for s in uniq if s["lanes"] == 3 and abs(s["road_offset_m"]) < 1e-6 and s["nav_lat_from_center"] > 0]
    stats_l3_ro0 = {
        "n": len(lanes3_ro0),
        "median_nav_lat": med([s["nav_lat_from_center"] for s in lanes3_ro0]),
        "median_offset_a": med([s["offset_a"] for s in lanes3_ro0]),
        "median_offset_b": med([s["offset_b"] for s in lanes3_ro0]),
        "median_err_a": med([s["err_a"] for s in lanes3_ro0]),
        "median_err_b": med([s["err_b"] for s in lanes3_ro0]),
    }

    # Top 10 best B matches (same sign, low err_b)
    best_b = sorted(same_sign, key=lambda s: s["err_b"])[:10]

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(
        json.dumps(
            {
                "stats": stats_all,
                "stats_road_offset_0": stats_ro0,
                "stats_road_offset_nonzero": stats_ro_nz,
                "stats_same_sign": stats_same,
                "stats_lanes3_ro0_right": stats_l3_ro0,
                "best_b_matches": best_b,
                "report_samples": report[:15],
                "all_count": len(uniq),
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(json.dumps({"stats": stats_all, "ro0": stats_ro0, "ro_nz": stats_ro_nz, "same": stats_same, "l3": stats_l3_ro0}, indent=2))
    print("--- report samples ---")
    for s in report[:12]:
        print(
            f"lanes={s['lanes']} ro={s['road_offset_m']} nav_lat={s['nav_lat_from_center']} "
            f"oa={s['offset_a']} ob={s['offset_b']} err_a={s['err_a']} err_b={s['err_b']} "
            f"dist_a={s['dist_nav_road_target_a']} dist_b={s['dist_nav_road_target_b']} "
            f"hdg_d={s['hdg_diff_deg']}"
        )


if __name__ == "__main__":
    main()
