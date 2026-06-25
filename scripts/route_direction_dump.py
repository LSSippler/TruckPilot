#!/usr/bin/env python3
"""
route_direction_dump.py — Live-Dump der Route-Richtungsbeziehung.

Beantwortet Kandidat A (router.waypoints-Ordnung) und liefert Rohmaterial fuer
B (route_node_ids / snap) und C (lane_keeper-Hop). Dumpt:
  - Truck-Position + Heading + Forward-Vektor (ETS2-Konvention)
  - router.waypoints[0..6] + letzte 3, mit dist, signed dz, Winkel-zu-Heading
  - z-Trend der Polyline (laeuft sie in oder gegen Fahrtrichtung?)
  - router.route_node_ids, snap-Diagnostik, goal
  - lane_keeper Heading-Diffs (route vs free-nearest)
  - Test: bringt +z-Weiterfahrt den Truck naeher an oder weiter weg vom Ziel?

Usage:
    python scripts/route_direction_dump.py --out outputs/2026-06-14/route_direction_dump.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone

import websocket  # type: ignore

DEFAULT_URL = "ws://127.0.0.1:8765"
TAU = math.tau

KEYS = [
    "autopilot.state",
    "telemetry.position_x", "telemetry.position_z", "telemetry.heading", "telemetry.speed_ms",
    "router.active", "router.last_planning_result", "router.waypoint_count",
    "router.path_total_distance_m",
    "router.waypoints", "router.route_node_ids",
    "router.current_goal_uid", "router.goal_uid",
    "router.last_snap_dist", "router.snap_method", "router.snap_stable_edge_id",
    "router.last_snap_rejected_by_heading", "router.last_snap_heading_filter_applied",
    "router.snap_window_unique_edges", "router.snap_stability",
    "lane_keeper.nearest_route_heading_diff_deg",
    "lane_keeper.engage_heading_diff_deg", "lane_keeper.engage_dist_m",
    "lane_keeper.fallback_reason", "lane_keeper.lateral_source",
    "lane_keeper.truck_lat_vs_centerline_m",
    "state.heading_stage", "state.engage_detail_heading_diff_deg",
    "state.engage_blocked_by",
]


def fnum(s):
    if s is None:
        return None
    try:
        return float(str(s).strip())
    except (ValueError, TypeError):
        return None


def fwd(heading):
    hr = -heading * TAU
    return (math.sin(hr), -math.cos(hr))


def angle_to(px, pz, wx, wz, heading):
    dx, dz = wx - px, wz - pz
    ln = math.hypot(dx, dz)
    if ln < 1e-6:
        return 0.0
    fx, fz = fwd(heading)
    dot = max(-1.0, min(1.0, fx * dx / ln + fz * dz / ln))
    return math.degrees(math.acos(dot))


def fetch(ws, keys):
    ws.send(json.dumps({"type": "blackboard_get", "keys": keys}))
    for _ in range(40):
        m = json.loads(ws.recv())
        if m.get("type") == "blackboard_snapshot":
            return m.get("values") or {}
        if m.get("type") == "error":
            raise RuntimeError(m.get("message"))
    raise RuntimeError("no snapshot")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ws = websocket.create_connection(args.url, timeout=5)
    v = fetch(ws, KEYS)
    ws.close()

    px, pz = fnum(v.get("telemetry.position_x")), fnum(v.get("telemetry.position_z"))
    heading = fnum(v.get("telemetry.heading"))
    fx, fz = fwd(heading) if heading is not None else (None, None)

    wps = None
    try:
        wps = json.loads(v.get("router.waypoints") or "")
    except (ValueError, TypeError):
        pass

    analysis = {
        "truck": {"x": px, "z": pz, "heading": heading, "fwd_x": fx, "fwd_z": fz,
                  "speed_ms": fnum(v.get("telemetry.speed_ms"))},
        "goal_uid": v.get("router.current_goal_uid"),
        "snap": {
            "last_snap_dist": v.get("router.last_snap_dist"),
            "snap_method": v.get("router.snap_method"),
            "snap_stable_edge_id": v.get("router.snap_stable_edge_id"),
            "rejected_by_heading": v.get("router.last_snap_rejected_by_heading"),
            "heading_filter_applied": v.get("router.last_snap_heading_filter_applied"),
            "snap_window_unique_edges": v.get("router.snap_window_unique_edges"),
        },
        "route_node_ids_raw": v.get("router.route_node_ids"),
        "waypoint_count": v.get("router.waypoint_count"),
        "path_total_distance_m": v.get("router.path_total_distance_m"),
        "lane_keeper": {
            "nearest_route_heading_diff_deg": v.get("lane_keeper.nearest_route_heading_diff_deg"),
            "engage_heading_diff_deg": v.get("lane_keeper.engage_heading_diff_deg"),
            "engage_dist_m": v.get("lane_keeper.engage_dist_m"),
            "fallback_reason": v.get("lane_keeper.fallback_reason"),
            "lateral_source": v.get("lane_keeper.lateral_source"),
            "truck_lat_vs_centerline_m": v.get("lane_keeper.truck_lat_vs_centerline_m"),
        },
        "state": {
            "autopilot.state": v.get("autopilot.state"),
            "heading_stage": v.get("state.heading_stage"),
            "engage_detail_heading_diff_deg": v.get("state.engage_detail_heading_diff_deg"),
            "engage_blocked_by": v.get("state.engage_blocked_by"),
        },
    }

    wp_rows = []
    if isinstance(wps, list) and px is not None:
        head = wps[:7]
        tail_start = max(7, len(wps) - 3)
        for idx, wp in enumerate(head):
            if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                wx, wz = float(wp[0]), float(wp[1])
                wp_rows.append({
                    "i": idx, "x": round(wx, 2), "z": round(wz, 2),
                    "dist_m": round(math.hypot(wx - px, wz - pz), 2),
                    "dz_from_truck": round(wz - pz, 2),
                    "angle_to_heading_deg": round(angle_to(px, pz, wx, wz, heading), 2),
                })
        for idx in range(tail_start, len(wps)):
            wp = wps[idx]
            if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                wx, wz = float(wp[0]), float(wp[1])
                wp_rows.append({
                    "i": idx, "x": round(wx, 2), "z": round(wz, 2),
                    "dist_m": round(math.hypot(wx - px, wz - pz), 2),
                    "dz_from_truck": round(wz - pz, 2),
                    "angle_to_heading_deg": round(angle_to(px, pz, wx, wz, heading), 2),
                    "TAIL": True,
                })
        # z-trend over first 10 segments
        zs = [float(w[1]) for w in wps[:11] if isinstance(w, (list, tuple)) and len(w) >= 2]
        dzs = [round(zs[i + 1] - zs[i], 1) for i in range(len(zs) - 1)]
        analysis["polyline_first10_dz"] = dzs
        analysis["polyline_z_first"] = round(float(wps[0][1]), 2)
        analysis["polyline_z_last"] = round(float(wps[-1][1]), 2)
        # does +z driving approach or leave the goal (=last wp)?
        gx, gz = float(wps[-1][0]), float(wps[-1][1])
        analysis["goal_xz"] = [round(gx, 2), round(gz, 2)]
        analysis["truck_to_goal_dist_m"] = round(math.hypot(gx - px, gz - pz), 1)
        analysis["truck_to_goal_dz"] = round(gz - pz, 1)
        analysis["heading_points_toward_goal_deg"] = round(angle_to(px, pz, gx, gz, heading), 2)
    analysis["waypoints"] = wp_rows

    report = {
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "analysis": analysis,
        "raw": v,
    }

    # console summary
    print("=== ROUTE DIRECTION DUMP ===")
    print(f"truck  x={px} z={pz} heading={heading}  fwd=({fx:.3f},{fz:.3f})" if heading is not None else "no heading")
    print(f"state={v.get('autopilot.state')}  blocked='{v.get('state.engage_blocked_by')}'")
    print(f"snap_dist={v.get('router.last_snap_dist')} method={v.get('router.snap_method')} "
          f"stable_edge={v.get('router.snap_stable_edge_id')} rej_by_heading={v.get('router.last_snap_rejected_by_heading')}")
    print(f"lane_keeper nearest_route_hdiff={v.get('lane_keeper.nearest_route_heading_diff_deg')} "
          f"engage_hdiff={v.get('lane_keeper.engage_heading_diff_deg')} engage_dist={v.get('lane_keeper.engage_dist_m')}")
    print(f"route_node_ids={ (v.get('router.route_node_ids') or '')[:120] }")
    print(f"polyline z: first={analysis.get('polyline_z_first')} last={analysis.get('polyline_z_last')} "
          f"first10_dz={analysis.get('polyline_first10_dz')}")
    print(f"goal_xz={analysis.get('goal_xz')} truck_to_goal_dist={analysis.get('truck_to_goal_dist_m')} "
          f"dz={analysis.get('truck_to_goal_dz')} heading_to_goal={analysis.get('heading_points_toward_goal_deg')}deg")
    print("waypoints (i, x, z, dist, dz_from_truck, angle_to_heading):")
    for r in wp_rows:
        tag = " <TAIL>" if r.get("TAIL") else ""
        print(f"  [{r['i']:>3}] x={r['x']:>10} z={r['z']:>10} dist={r['dist_m']:>8} "
              f"dz={r['dz_from_truck']:>9} ang={r['angle_to_heading_deg']:>7}{tag}")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
