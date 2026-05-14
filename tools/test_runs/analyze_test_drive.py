#!/usr/bin/env python3
"""
analyze_test_drive.py — reads stats.db from a TruckPilot test drive and
prints human-readable summary statistics.

Usage:
  python3 analyze_test_drive.py stats.db [--phase N]
  python3 analyze_test_drive.py docs/test_runs/2026-05-11_phase2_smoke/stats.db
"""

import argparse
import sqlite3
import sys
from pathlib import Path


def connect(db_path: str) -> sqlite3.Connection:
    if not Path(db_path).exists():
        print(f"ERROR: {db_path} not found")
        sys.exit(1)
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def analyze_sessions(conn: sqlite3.Connection, label: str = "") -> dict:
    rows = conn.execute("SELECT * FROM sessions ORDER BY id").fetchall()
    if not rows:
        print("  (no sessions)")
        return {}
    for s in rows:
        dur_min = s["duration_s"] / 60.0
        avg = s["avg_speed_kmh"] or 0.0
        fuel = s["fuel_used_l"] or 0.0
        print(f"  Session #{s['id']}: {s['distance_km']:.1f} km in {dur_min:.0f} min "
              f"({avg:.1f} km/h avg), {fuel:.1f} L fuel, {s['pauses']} pauses")
    last = rows[-1]
    return {
        "total_distance_km": last["distance_km"],
        "total_duration_s": last["duration_s"],
        "avg_speed_kmh": last["avg_speed_kmh"] or 0.0,
        "fuel_used_l": last["fuel_used_l"] or 0.0,
    }


def analyze_tick_log(conn: sqlite3.Connection) -> dict:
    """Analyse tick_log und extrahiere Speed- und Lane-Keeping-Statistiken."""
    rows = conn.execute(
        "SELECT * FROM tick_log ORDER BY timestamp_ms"
    ).fetchall()
    if not rows:
        print("  (no tick_log data)")
        return {}

    # State distribution
    state_secs = {}
    total = len(rows)
    prev_ts = rows[0]["timestamp_ms"]
    prev_state = rows[0]["autopilot_state"]
    for r in rows[1:]:
        dt = (r["timestamp_ms"] - prev_ts) / 1000.0
        if dt > 10.0:
            dt = 0.02  # clamp gap (teleport, pause)
        state_secs[prev_state] = state_secs.get(prev_state, 0.0) + dt
        prev_ts = r["timestamp_ms"]
        prev_state = r["autopilot_state"]
    # Add last segment
    state_secs[prev_state] = state_secs.get(prev_state, 0.0) + 0.02

    total_duration = sum(state_secs.values())

    print(f"\n  AUTOPILOT STATES ({total} samples, {total_duration:.1f} s):")
    for s, dur in sorted(state_secs.items(), key=lambda kv: -kv[1]):
        pct = 100.0 * dur / max(total_duration, 0.001)
        print(f"    {s:<12} {dur:>8.1f} s ({pct:5.1f} %)")

    # Interventions
    interventions = [r for r in rows if r["intervention"]]
    print(f"\n  INTERVENTIONS: {len(interventions)}")
    for inv in interventions[:20]:
        ts = inv["timestamp_ms"] / 1000.0
        print(f"    @ {ts:>7.1f}s  state={inv['autopilot_state']}")

    # Speed control
    speed_rows = [r for r in rows if r["speed_kmh"] is not None and r["target_speed_kmh"] is not None]
    speed_errors = []
    deadband_count = 0
    for r in speed_rows:
        err = r["speed_kmh"] - r["target_speed_kmh"]
        speed_errors.append(err)
        if abs(err) < 1.0:
            deadband_count += 1

    if speed_errors:
        n = len(speed_errors)
        rms = (sum(e * e for e in speed_errors) / n) ** 0.5
        sorted_abs = sorted(abs(e) for e in speed_errors)
        p95 = sorted_abs[int(n * 0.95)] if n > 20 else sorted_abs[-1]
        dead_pct = 100.0 * deadband_count / n

        print(f"\n  SPEED CONTROL ({n} samples):")
        print(f"    Speed Error (RMS):          {rms:.2f} km/h")
        print(f"    Speed Error (95th):         {p95:.2f} km/h")
        print(f"    Time in Dead-Band:          {dead_pct:.1f} %")
        print(f"    Target Speed Range:         {min(r['target_speed_kmh'] for r in speed_rows):.0f} - {max(r['target_speed_kmh'] for r in speed_rows):.0f} km/h")

    # Sign limits
    sign_rows = [r for r in rows if r["sign_limit_kmh"] is not None]
    if sign_rows:
        limits = sorted(set(r["sign_limit_kmh"] for r in sign_rows))
        print(f"\n  SIGN LIMITS seen: {', '.join(f'{v:.0f}' for v in limits)} km/h")

    return {
        "total_samples": total,
        "state_secs": state_secs,
        "intervention_count": len(interventions),
        "speed_rms_kmh": (sum(e * e for e in speed_errors) / len(speed_errors)) ** 0.5 if speed_errors else None,
    }


def analyze_fault_log(conn: sqlite3.Connection) -> list:
    rows = conn.execute(
        "SELECT * FROM fault_log ORDER BY timestamp_ms"
    ).fetchall()
    print(f"\n  FAULT LOG: {len(rows)} entries")
    for f in rows:
        ts = f["timestamp_ms"] / 1000.0
        print(f"    @ {ts:>7.1f}s  reason={f['reason']}")
    return rows


def analyze_pid_tuning(conn: sqlite3.Connection) -> list:
    rows = conn.execute(
        "SELECT * FROM pid_tuning_log ORDER BY timestamp_ms"
    ).fetchall()
    print(f"\n  PID TUNING CHANGES: {len(rows)}")
    for t in rows:
        ts = t["timestamp_ms"] / 1000.0
        print(f"    [{ts:>7.1f}s] {t['plugin_name']}.{t['parameter']}: "
              f"{t['old_value']:.3f} -> {t['new_value']:.3f}  ({t['set_by']})")
    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Analyse a TruckPilot test-drive stats.db"
    )
    parser.add_argument("db_path", help="Path to stats.db")
    parser.add_argument(
        "--phase", type=str, default="",
        help="Optional phase label (e.g. 'Phase 1 — Smoke')"
    )
    args = parser.parse_args()

    phase = args.phase or Path(args.db_path).parent.name

    print("=" * 60)
    print(f"  TEST DRIVE ANALYSIS — {phase}")
    print("=" * 60)

    conn = sqlite3.connect(args.db_path)
    conn.row_factory = sqlite3.Row

    # 1. Sessions
    print("\n--- SESSIONS ---")
    session = analyze_sessions(conn)

    if session:
        print(f"\n  SUMMARY:")
        print(f"    Duration:      {session['total_duration_s']:.1f} s "
              f"({session['total_duration_s'] / 60:.1f} min)")
        print(f"    Distance:      {session['total_distance_km']:.1f} km")
        print(f"    Avg Speed:     {session['avg_speed_kmh']:.1f} km/h")

    # 2. Tick log
    print("\n--- TICK LOG ---")
    tick = analyze_tick_log(conn)

    # 3. Fault log
    print("\n--- FAULT LOG ---")
    analyze_fault_log(conn)

    # 4. PID tuning
    print("\n--- PID TUNING LOG ---")
    analyze_pid_tuning(conn)

    conn.close()
    print(f"\n{'='*60}")
    print("  Analysis complete.")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
