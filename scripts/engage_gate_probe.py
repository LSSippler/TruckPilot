#!/usr/bin/env python3
"""
engage_gate_probe.py — Live-Rohwert-Probe fuer die drei Engage-Gates.

Liest die Blackboard-Keys, die in
  crates/core/src/state_machine.rs::evaluate_engagement_preconditions
die drei Gates speisen, UND rechnet die Gate-Geometrie (Truck-Heading vs
Richtung-zu-waypoints[1]) lokal nach — exakt mit derselben Konvention wie
compute_heading_aligned / heading_diff_degrees im Rust-Code.

Ziel: empirisch belegen, WELCHER Rohwert WELCHES Gate reisst und ob das fuer
das Gate verwendete Segment (router.waypoints[1]) zur echten Fahrtrichtung
passt — im Vergleich zu lane_keeper.engage_heading_diff_deg (Segment-Tangente).

Usage:
    python scripts/engage_gate_probe.py --hz 5 --secs 4 --out outputs/2026-06-14/engage_gate_probe.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone

import websocket  # type: ignore

DEFAULT_URL = "ws://127.0.0.1:8765"

KEYS = [
    # --- live truth ---
    "autopilot.state",
    "telemetry.position_x",
    "telemetry.position_z",
    "telemetry.heading",
    "telemetry.speed_ms",
    # --- aggregate gate state ---
    "state.engage_ready",
    "state.engage_all_ok",
    "state.engage_blocked_by",
    "state.engage_advisory",
    # --- per-gate booleans (already published) ---
    "state.engage_precondition_telemetry_fresh",
    "state.engage_precondition_truck_on_road",
    "state.engage_precondition_heading_aligned",
    "state.engage_precondition_route_planned",
    "state.engage_precondition_truck_on_route",
    "state.engage_precondition_speed_ok",
    "state.engage_precondition_heading_ok_for_engage",
    "state.engage_precondition_lane_keeper_engage_allowed",
    # --- gate raw values that DO exist ---
    "state.engage_detail_heading_diff_deg",   # gate heading measure (vs wps[1])
    "state.engage_detail_snap_dist_m",
    "state.engage_detail_speed_kmh",
    "state.heading_diff_deg",
    # --- router inputs that feed the gates ---
    "router.active",
    "router.last_planning_result",
    "router.last_snap_dist",
    "router.last_snap_edge_id",     # truck_on_route input #1
    "router.route_edge_ids",        # truck_on_route input #2
    "router.current_goal_uid",
    "router.graph_node_count",
    "router.waypoints",             # heading_aligned / heading_ok geometry source
    # --- lane-keeper measure (DIFFERENT segment: tangent of nearest spline) ---
    "plugin.lane_keeper.mode",
    "lane_keeper.mode",
    "lane_keeper.engage_allowed",
    "lane_keeper.engage_dist_m",
    "lane_keeper.engage_heading_diff_deg",
    "lane_keeper.nearest_route_heading_diff_deg",
    "lane_keeper.truck_to_segment_dist_m",
    "lane_keeper.diag_truck_to_segp0_m",
    "lane_keeper.active",
]

TAU = math.tau


def fnum(s):
    if s is None:
        return None
    try:
        return float(str(s).strip())
    except ValueError:
        return None


def forward_vec(heading: float) -> tuple[float, float]:
    """Exact ETS2 convention from state_machine.rs."""
    hr = -heading * TAU
    return (math.sin(hr), -math.cos(hr))


def chord_angle_deg(px, pz, wx, wz, heading) -> float | None:
    dx = wx - px
    dz = wz - pz
    ln = math.hypot(dx, dz)
    if ln < 0.01:
        return 0.0
    fx, fz = forward_vec(heading)
    dot = (fx * dx / ln) + (fz * dz / ln)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def fetch(ws, keys):
    ws.send(json.dumps({"type": "blackboard_get", "keys": keys}))
    for _ in range(40):
        msg = json.loads(ws.recv())
        if msg.get("type") == "blackboard_snapshot":
            return msg.get("values") or {}
        if msg.get("type") == "error":
            raise RuntimeError(msg.get("message", "daemon error"))
    raise RuntimeError("no blackboard_snapshot reply")


def analyze(v: dict) -> dict:
    """Recompute the gate geometry from raw telemetry + waypoints."""
    out = {}
    px = fnum(v.get("telemetry.position_x"))
    pz = fnum(v.get("telemetry.position_z"))
    heading = fnum(v.get("telemetry.heading"))
    out["px"], out["pz"], out["heading"] = px, pz, heading

    wps = None
    raw = v.get("router.waypoints")
    if raw:
        try:
            wps = json.loads(raw)
        except (ValueError, TypeError):
            wps = None
    out["waypoint_count"] = len(wps) if isinstance(wps, list) else None

    # Gate geometry: angle truck-heading vs direction to wps[1], wps[0], wps[2]
    if isinstance(wps, list) and px is not None and pz is not None and heading is not None:
        for idx in (0, 1, 2):
            if len(wps) > idx and isinstance(wps[idx], (list, tuple)) and len(wps[idx]) >= 2:
                ang = chord_angle_deg(px, pz, float(wps[idx][0]), float(wps[idx][1]), heading)
                dist = math.hypot(float(wps[idx][0]) - px, float(wps[idx][1]) - pz)
                out[f"recomputed_angle_to_wp{idx}_deg"] = round(ang, 2) if ang is not None else None
                out[f"dist_to_wp{idx}_m"] = round(dist, 2)
        a1 = out.get("recomputed_angle_to_wp1_deg")
        if a1 is not None:
            out["gate_heading_aligned_recomputed"] = a1 <= 45.0     # dot>=0.707
            out["gate_heading_ok_for_engage_recomputed"] = a1 < 60.0

    # truck_on_route: pure set membership, NO geometry
    snap_edge = v.get("router.last_snap_edge_id")
    route_set = v.get("router.route_edge_ids")
    out["truck_on_route_snap_edge"] = snap_edge if snap_edge not in (None, "") else "<ABSENT>"
    out["truck_on_route_route_set"] = (
        (route_set[:60] + "...") if isinstance(route_set, str) and len(route_set) > 60
        else (route_set if route_set not in (None, "") else "<ABSENT>")
    )
    if snap_edge in (None, "") or route_set in (None, ""):
        out["gate_truck_on_route_recomputed"] = False
        out["truck_on_route_reason"] = "dead_key(s)_absent -> match _=>false"
    else:
        members = {s.strip() for s in str(route_set).split(",")}
        out["gate_truck_on_route_recomputed"] = str(snap_edge).strip() in members
        out["truck_on_route_reason"] = "computed_from_membership"
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ws = websocket.create_connection(args.url, timeout=5)
    samples = []
    n = max(1, int(args.hz * args.secs))
    period = 1.0 / args.hz
    print(f"# engage_gate_probe — {n} Samples @ {args.hz} Hz")
    for i in range(n):
        v = fetch(ws, KEYS)
        a = analyze(v)
        samples.append({"raw": v, "derived": a})
        gb = v.get("state.engage_blocked_by", "")
        print(
            f"[{i:02d}] state={v.get('autopilot.state'):<12} "
            f"spd={fnum(v.get('telemetry.speed_ms')) or 0:.1f}m/s "
            f"blocked='{gb}'"
        )
        print(
            f"     GATE-heading_to_wp1={a.get('recomputed_angle_to_wp1_deg')}deg "
            f"(detail_key={v.get('state.engage_detail_heading_diff_deg')}) "
            f"| LANE-tangent={v.get('lane_keeper.engage_heading_diff_deg')}deg "
            f"dist={v.get('lane_keeper.engage_dist_m')}m"
        )
        print(
            f"     truck_on_route: snap_edge={a.get('truck_on_route_snap_edge')} "
            f"route_set={a.get('truck_on_route_route_set')} "
            f"-> {a.get('gate_truck_on_route_recomputed')} ({a.get('truck_on_route_reason')})"
        )
        if i + 1 < n:
            time.sleep(period)
    ws.close()

    report = {
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "url": args.url,
        "sample_count": len(samples),
        "samples": samples,
    }
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
