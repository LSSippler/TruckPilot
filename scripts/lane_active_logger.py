#!/usr/bin/env python3
"""
lane_active_logger.py — Gefuehrter Active-Lauf + Live-Log fuer Lane-Geometrie.

Fuehrt dich Schritt fuer Schritt durch den stabilen Test:
  1) Vorbereitung (g_traffic 0, gerade Strecke)
  2) Route/Daemon ok
  3) ~50 km/h mittig fahren
  4) Stabilitaet bestaetigen
  5) Engage -> sofort loggen waehrend Active

Usage:
    pip install websocket-client
    python scripts/lane_active_logger.py
    python scripts/lane_active_logger.py --auto-engage
    python scripts/lane_active_logger.py --no-guided --active-only

Strg+C = Stopp + Auswertung.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import deque
from datetime import datetime, timezone
from enum import Enum, auto
from pathlib import Path
from statistics import mean
from typing import Any

DEFAULT_URL = "ws://127.0.0.1:8765"

SETUP_KEYS = [
    "telemetry.available",
    "telemetry.engine_rpm",
    "telemetry.speed_ms",
    "telemetry.heading",
    "router.active",
    "router.graph_node_count",
    "router.current_goal_uid",
    "router.last_planning_result",
    "state.engage_ready",
    "state.engage_blocked_by",
    "state.heading_diff_deg",
    "autopilot.state",
    "lane_keeper.engage_allowed",
    "lane_keeper.engage_dist_m",
    "lane_keeper.engage_heading_diff_deg",
]

LOG_KEYS = [
    "autopilot.state",
    "autopilot.fault_reason",
    "state.state_age_ticks",
    "state.last_transition",
    "state.last_disengage_reason",
    "state.heading_stage",
    "telemetry.speed_ms",
    "telemetry.heading",
    "telemetry.engine_gear",
    "telemetry.reverse_gear",
    "telemetry.accel_longitudinal",
    "cruise.target_kmh",
    "state.engage_ready",
    "state.engage_blocked_by",
    "router.active",
    "daemon.plugin_tick_count",
    "lane_keeper.active",
    "lane_keeper.skip_reason",
    "lane_keeper.stage",
    "lane_keeper.stage_block_reason",
    "lane_keeper.steering_request_emitted",
    "state.heading_diff_deg",
    "state.heading_stage_desired",
    "state.heading_stage_hysteresis_ticks",
    "router.auto_replan_count",
    "lane_keeper.tick_seq",
    "lane_keeper.last_tick_us",
    "lane_keeper.tick_ctx_active",
    "lane_keeper.mode",
    "lane_keeper.fallback_reason",
    "lane_keeper.lateral_source",
    "lane_keeper.truck_lat_vs_centerline_m",
    "lane_keeper.truck_lat_vs_offsetline_m",
    "lane_keeper.lane_offset_applied_m",
    "lane_keeper.nearest_route_heading_diff_deg",
    "lane_keeper.diag_truck_to_segp0_m",
    "lane_keeper.steer_raw",
    "lane_keeper.heading_error_rad",
    "lane_keeper.truck_to_segment_dist_m",
    "speed_controller.tick_seq",
    "speed_controller.tick_ctx_active",
    "speed_controller.throttle_cmd",
    "speed_controller.brake_cmd",
    "speed_controller.target_speed_kmh",
    "speed_controller.target_speed_ms",
    "speed_controller.speed_error_ms",
    "speed_controller.speed_error_kmh",
    "speed_controller.target_limiting_source",
    "speed_controller.cruise_target_kmh",
    "acc.tick_seq",
    "acc.active",
    "acc.speed_cap_kmh",
    "acc.throttle_cap_kmh",
    "acc.target_speed_ms",
    "acc.brake_cmd",
    "acc.lead_vehicle_detected",
    "acc.lead_vehicle_distance_m",
    "acc.using_accel_proxy",
    "arbitration.steering_winner_plugin",
    "arbitration.steering_offer_count",
    "arbitration.steering_value",
    "arbitration.throttle_winner_plugin",
    "arbitration.brake_winner_plugin",
    "arbitration.longitudinal_winner_plugin",
    "arbitration.longitudinal_throttle_final",
    "arbitration.longitudinal_brake_final",
    "arbitration.throttle_value",
    "arbitration.brake_value",
    "scs_sdk_output.init_state",
    "scs_sdk_output.last_error",
    "scs_sdk_output.connected",
    "plugin.vjoy_output.enabled",
    "plugin.scs_sdk_output.enabled",
    "plugin.vjoy_output.dll_loaded",
    "plugin.scs_sdk_output.dll_loaded",
    "vjoy.init_error",
    "output.sink.configured",
    "scs_sdk_output.active",
    "scs_sdk_output.steer_written",
    "scs_sdk_output.throttle_written",
    "scs_sdk_output.brake_written",
    "scs_sdk_output.last_write_tick",
    "vjoy.connected",
    "vjoy.last_raw_x",
    "vjoy.last_raw_sl0",
    "vjoy.last_raw_sl1",
    "vjoy.last_write_tick",
    "vjoy.last_raw_source",
]

SKIP_REPLY_TYPES = frozenset({"hello", "plugin_list"})


class Phase(Enum):
    BRIEFING = auto()
    WAIT_ROUTE = auto()
    WAIT_DRIVE = auto()
    WAIT_STABLE = auto()
    ENGAGE = auto()
    LOGGING = auto()
    DONE = auto()


PHASE_LABEL = {
    Phase.BRIEFING: "1/5 Vorbereitung",
    Phase.WAIT_ROUTE: "2/5 Route",
    Phase.WAIT_DRIVE: "3/5 Fahren (~50 km/h, mittig)",
    Phase.WAIT_STABLE: "4/5 Stabil halten",
    Phase.ENGAGE: "5/5 ENGAGE",
    Phase.LOGGING: "LOG Active",
    Phase.DONE: "Fertig",
}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Gefuehrter Lane-Active Logger")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--hz", type=float, default=5.0, help="Poll-Rate waehrend Log (default 5/s)")
    ap.add_argument("--no-guided", action="store_true", help="Altes Verhalten: sofort loggen")
    ap.add_argument("--active-only", action="store_true", help="Nur bei guided implizit in LOG-Phase")
    ap.add_argument(
        "--auto-engage",
        action="store_true",
        help="Engage per WebSocket senden sobald stabil (sonst ENTER oder UI)",
    )
    ap.add_argument(
        "--lane-only",
        action="store_true",
        help="Lane-only Engage (keine Route noetig; nearest_spline, engage_allowed-Gate)",
    )
    ap.add_argument("--speed-min-kmh", type=float, default=40.0)
    ap.add_argument("--speed-max-kmh", type=float, default=62.0)
    ap.add_argument(
        "--stable-sec",
        type=float,
        default=2.0,
        help="Sekunden stabile Geschwindigkeit vor Engage-Prompt",
    )
    ap.add_argument(
        "--log-sec",
        type=float,
        default=8.0,
        help="Active loggen N Sekunden dann Stopp (0=Strg+C)",
    )
    ap.add_argument("--out", type=Path, help="CSV-Pfad")
    ap.add_argument("--seconds", type=float, default=0.0, help="Legacy: max Laufzeit (--no-guided)")
    return ap.parse_args()


def connect_ws(url: str):
    try:
        import websocket
    except ImportError:
        print("Fehlt: pip install websocket-client", file=sys.stderr)
        sys.exit(1)
    ws = websocket.create_connection(url, timeout=5)
    ws.settimeout(2.0)
    return ws


def recv_until(ws, accept_types: frozenset[str]) -> dict[str, Any]:
    for _ in range(40):
        raw = ws.recv()
        msg: dict[str, Any] = json.loads(raw)
        t = msg.get("type")
        if t in SKIP_REPLY_TYPES:
            continue
        if t in accept_types:
            return msg
        if t == "error":
            raise RuntimeError(msg.get("message", "daemon error"))
    raise RuntimeError(f"keine Antwort ({accept_types})")


def fetch_keys(ws, keys: list[str]) -> dict[str, str]:
    ws.send(json.dumps({"type": "blackboard_get", "keys": keys}))
    msg = recv_until(ws, frozenset({"blackboard_snapshot"}))
    return msg.get("values") or {}


def send_cmd(ws, cmd: dict[str, Any]) -> None:
    ws.send(json.dumps(cmd))


def request_engage(ws, lane_only: bool = False) -> None:
    if lane_only:
        send_cmd(
            ws,
            {
                "type": "set_blackboard_key",
                "key": "autopilot.requested_mode",
                "value": "lane_only",
            },
        )
        time.sleep(0.05)
    send_cmd(ws, {"type": "autopilot_engage"})
    time.sleep(0.2)


def fnum(s: str | None) -> float | None:
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def ms_to_kmh(v: float) -> float:
    return v * 3.6


def default_out_path() -> Path:
    today = datetime.now().strftime("%Y-%m-%d")
    stamp = datetime.now().strftime("%H%M%S")
    return Path(f"outputs/{today}/lane-active-{stamp}.csv")


def print_banner(title: str, lines: list[str]) -> None:
    bar = "=" * 60
    print(f"\n{bar}\n  {title}\n{bar}")
    for line in lines:
        print(f"  {line}")
    print(bar)


def print_status(phase: Phase, hint: str, values: dict[str, str]) -> None:
    spd = fnum(values.get("telemetry.speed_ms"))
    kmh = f"{ms_to_kmh(spd):.0f} km/h" if spd is not None else "? km/h"
    ready = values.get("state.engage_ready", "?")
    blocked = values.get("state.engage_blocked_by", "") or "-"
    router = values.get("router.active", "?")
    state = values.get("autopilot.state", "?")
    print(
        f"[{PHASE_LABEL[phase]}] {hint}\n"
        f"    speed={kmh}  engage_ready={ready}  blocked={blocked}\n"
        f"    router.active={router}  autopilot.state={state}"
    )


def speed_in_band(values: dict[str, str], lo_kmh: float, hi_kmh: float) -> bool:
    spd = fnum(values.get("telemetry.speed_ms"))
    if spd is None:
        return False
    kmh = ms_to_kmh(spd)
    rpm = fnum(values.get("telemetry.engine_rpm")) or 0.0
    return lo_kmh <= kmh <= hi_kmh and rpm > 100.0


def fmt_log_line(values: dict[str, str], elapsed: float) -> str:
    lat = values.get("lane_keeper.truck_lat_vs_centerline_m", "-")
    seg0 = values.get("lane_keeper.diag_truck_to_segp0_m", "-")
    steer = values.get("lane_keeper.steer_raw", "-")
    tick_seq = values.get("lane_keeper.tick_seq", "-")
    skip = values.get("lane_keeper.skip_reason", "-")
    thr = values.get("speed_controller.throttle_cmd", "-")
    brk = values.get("speed_controller.brake_cmd", "-")
    tgt = values.get("speed_controller.target_speed_kmh", "-")
    acc = values.get("acc.active", "-")
    spd = fnum(values.get("telemetry.speed_ms"))
    kmh = f"{ms_to_kmh(spd):.0f}" if spd else "?"
    gear = values.get("telemetry.engine_gear", "?")
    rev = values.get("telemetry.reverse_gear", "?")
    return (
        f"[{elapsed:5.1f}s ACTIVE] tick={tick_seq} skip={skip} "
        f"lat={lat} steer={steer} spd={kmh} gear={gear} rev={rev} "
        f"thr={thr} brk={brk} tgt={tgt} acc={acc}"
    )


def print_summary(
    active_lat: list[float],
    active_segp0: list[float],
    rows: int,
    tick_seqs: list[int] | None = None,
    disengage: dict[str, str] | None = None,
) -> None:
    print(f"\nActive-Zeilen: {rows}")
    if tick_seqs:
        if len(tick_seqs) >= 2:
            delta = tick_seqs[-1] - tick_seqs[0]
            frozen = len(set(tick_seqs)) == 1
            print("\n=== lane_keeper.tick_seq (Task 1: tickt im Active?) ===")
            print(f"  samples={len(tick_seqs)}  first={tick_seqs[0]}  last={tick_seqs[-1]}  delta={delta}")
            if frozen:
                print("  -> FROZEN: Lane-Keeper tick_request laeuft NICHT (Scheduling/Plugin aus)")
            elif delta >= len(tick_seqs) - 1:
                print("  -> OK: tick_seq zaehlt hoch (~1/Tick bei 5 Hz Poll erwartet delta << samples)")
            else:
                print("  -> TEILWEISE: tick_seq bewegt sich langsam — Poll-Rate vs Daemon-Tick pruefen")
        else:
            print("\n=== lane_keeper.tick_seq: zu wenige Active-Samples ===")
    if disengage:
        print("\n=== Disengage (Task 3) ===")
        print(f"  last_transition={disengage.get('state.last_transition', '?')}")
        print(f"  last_disengage_reason={disengage.get('state.last_disengage_reason', '?')}")
        print(f"  state_age_ticks={disengage.get('state.state_age_ticks', '?')} (~x20ms in Active)")
        if disengage.get("arbitration.steering_winner_plugin"):
            print("\n=== Output-Kette (Task 2) ===")
            print(f"  steering_winner={disengage.get('arbitration.steering_winner_plugin', '?')}")
            print(f"  steering_value={disengage.get('arbitration.steering_value', '?')}")
            print(f"  scs_sdk.active={disengage.get('scs_sdk_output.active', '?')}  steer_written={disengage.get('scs_sdk_output.steer_written', '?')}")
            print(f"  vjoy.last_raw_x={disengage.get('vjoy.last_raw_x', '?')}")
            print(f"  speed_controller.throttle_cmd={disengage.get('speed_controller.throttle_cmd', '?')}  tick_seq={disengage.get('speed_controller.tick_seq', '?')}")
    if active_lat:
        print("\n=== truck_lat_vs_centerline_m (du mittig auf Spur?) ===")
        print(
            f"  n={len(active_lat)}  min={min(active_lat):+.3f}  "
            f"max={max(active_lat):+.3f}  mean={mean(active_lat):+.3f} m"
        )
        if abs(mean(active_lat)) < 1.0:
            print("  -> |mean| < 1 m: Geometrie OK (12 m vorher eher Engage/Schraeglage)")
        else:
            print("  -> |mean| >= 1 m: Spline evtl. NEBEN der Straße (Parser/Offset)")
    else:
        print("\nKeine Active-Daten — Engage zu kurz oder nicht erreicht.")

    if active_segp0:
        print("\n=== diag_truck_to_segp0_m ===")
        print(
            f"  n={len(active_segp0)}  min={min(active_segp0):.2f}  "
            f"max={max(active_segp0):.2f}  mean={mean(active_segp0):.2f} m"
        )


def copy_to_claude(out_path: Path) -> None:
    claude_dir = Path("outputs/claude")
    claude_dir.mkdir(parents=True, exist_ok=True)
    dest = claude_dir / out_path.name
    dest.write_bytes(out_path.read_bytes())
    print(f"Kopie: {dest}")


def run_guided(args: argparse.Namespace) -> int:
    out_path = args.out or default_out_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    interval = 1.0 / max(args.hz, 0.1)
    all_keys = sorted(set(SETUP_KEYS + LOG_KEYS))

    mode_hint = (
        "Lane-only: KEINE Route noetig (engage_allowed=true, ~50 km/h Autobahn)"
        if args.lane_only
        else "Route-Modus: Ziel gesetzt (router.active=true)"
    )
    print_banner(
        "Lane-Active Test — Anleitung",
        [
            "ETS2: g_traffic 0, lange GERADE Autobahn",
            mode_hint,
            "Per HAND: mittig in Spur, geradeaus, ~50 km/h, Gas HALTEN",
            "Dieses Fenster offen lassen — Skript fuehrt dich durch",
            "",
            "Strg+C bricht jederzeit ab.",
        ],
    )
    input("\n  ENTER = Daemon verbinden und starten...\n")

    try:
        ws = connect_ws(args.url)
    except Exception as e:
        print(f"Verbindung fehlgeschlagen: {e}")
        return 1

    phase = Phase.BRIEFING
    last_phase_print = 0.0
    stable_since: float | None = None
    engage_sent = False
    log_started: float | None = None
    active_lat: list[float] = []
    active_segp0: list[float] = []
    active_tick_seqs: list[int] = []
    active_rows = 0
    last_values: dict[str, str] = {}
    t0 = time.monotonic()
    speed_hist: deque[float] = deque(maxlen=20)

    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["iso_time", "elapsed_s", "phase"] + LOG_KEYS)

        try:
            while phase != Phase.DONE:
                elapsed = time.monotonic() - t0
                try:
                    values = fetch_keys(ws, all_keys)
                except Exception as e:
                    print(f"  [!] Verbindung: {e} — reconnect...")
                    time.sleep(1.0)
                    try:
                        ws.close()
                    except Exception:
                        pass
                    ws = connect_ws(args.url)
                    continue

                state = values.get("autopilot.state", "")
                last_values = values
                spd = fnum(values.get("telemetry.speed_ms"))
                if spd is not None:
                    speed_hist.append(ms_to_kmh(spd))

                # --- State machine ---
                if phase == Phase.BRIEFING:
                    hint = (
                        "Pruefe Daemon + Telemetrie (lane-only, keine Route)."
                        if args.lane_only
                        else "Pruefe Daemon + Telemetrie. Route muss aktiv sein."
                    )
                    print_status(phase, hint, values)
                    tel = values.get("telemetry.available") == "true"
                    if not tel:
                        print("  WARTE: telemetry.available=false (Truck in Welt? DLL?)")
                    elif args.lane_only:
                        allowed = values.get("lane_keeper.engage_allowed") == "true"
                        if not allowed:
                            dist = values.get("lane_keeper.engage_dist_m", "?")
                            hdg = values.get("lane_keeper.engage_heading_diff_deg", "?")
                            print(
                                "  WARTE: lane_keeper.engage_allowed=false\n"
                                f"  dist={dist} m  heading_diff={hdg} deg\n"
                                "  -> mittig auf Spur fahren, Richtung anpassen"
                            )
                        else:
                            print("\n  OK: engage_allowed=true -> jetzt fahren (~50 km/h)")
                            phase = Phase.WAIT_DRIVE
                    elif values.get("router.active") != "true":
                        print(
                            "  WARTE: router.active=false\n"
                            "  -> Ziel in UI / engage-cli set-goal-pos\n"
                            "  -> ODER: --lane-only (routerlos, nearest_spline)"
                        )
                        phase = Phase.WAIT_ROUTE
                    else:
                        phase = Phase.WAIT_DRIVE
                    last_phase_print = elapsed

                elif phase == Phase.WAIT_ROUTE:
                    if elapsed - last_phase_print > 3.0:
                        print_status(phase, "Warte auf Route...", values)
                        last_phase_print = elapsed
                    if values.get("router.active") == "true":
                        print("\n  OK: Route aktiv -> jetzt fahren (~50 km/h, mittig)")
                        phase = Phase.WAIT_DRIVE
                        last_phase_print = elapsed

                elif phase == Phase.WAIT_DRIVE:
                    if elapsed - last_phase_print > 2.0:
                        hint = "Gas halten, mittig, geradeaus"
                        if speed_in_band(values, args.speed_min_kmh, args.speed_max_kmh):
                            hint += " — Geschwindigkeit OK"
                        else:
                            spd_kmh = ms_to_kmh(spd) if spd else 0.0
                            hint += f" — Ziel {args.speed_min_kmh:.0f}-{args.speed_max_kmh:.0f} km/h (jetzt {spd_kmh:.0f})"
                        print_status(phase, hint, values)
                        last_phase_print = elapsed
                    if speed_in_band(values, args.speed_min_kmh, args.speed_max_kmh):
                        stable_since = time.monotonic()
                        phase = Phase.WAIT_STABLE
                        print("\n  Geschwindigkeit im Band -> Stabilitaet pruefen (2 s)...")

                elif phase == Phase.WAIT_STABLE:
                    in_band = speed_in_band(values, args.speed_min_kmh, args.speed_max_kmh)
                    if not in_band:
                        stable_since = None
                        phase = Phase.WAIT_DRIVE
                        print("\n  Geschwindigkeit verlassen -> weiter fahren")
                        continue
                    assert stable_since is not None
                    stable_elapsed = time.monotonic() - stable_since
                    if elapsed - last_phase_print > 1.0:
                        print_status(
                            phase,
                            f"Stabil {stable_elapsed:.1f}/{args.stable_sec:.1f} s — Gas halten!",
                            values,
                        )
                        last_phase_print = elapsed
                    if stable_elapsed >= args.stable_sec:
                        phase = Phase.ENGAGE
                        ready = values.get("state.engage_ready") == "true"
                        blocked = values.get("state.engage_blocked_by", "")
                        allowed = values.get("lane_keeper.engage_allowed") == "true"
                        can_engage = allowed if args.lane_only else ready
                        gate_label = (
                            f"engage_allowed={allowed}  dist={values.get('lane_keeper.engage_dist_m', '?')} m"
                            if args.lane_only
                            else f"engage_ready={ready}  blocked={blocked or '-'}"
                        )
                        print_banner(
                            "JETZT ENGAGE",
                            [
                                gate_label,
                                "Gas weiter halten, mittig bleiben!",
                                "",
                                "Option A: Engage in TruckPilot-UI"
                                + (" (lane-only)" if args.lane_only else ""),
                                "Option B: ENTER hier (sendet engage)",
                                f"Option C: --auto-engage ({'AN' if args.auto_engage else 'aus'})",
                            ],
                        )
                        if args.auto_engage and can_engage:
                            request_engage(ws, args.lane_only)
                            engage_sent = True
                            mode = "lane-only" if args.lane_only else "route"
                            print(f"  -> engage gesendet ({mode}, --auto-engage)")
                            phase = Phase.LOGGING
                            log_started = time.monotonic()
                        elif args.auto_engage and not ready:
                            print(f"  auto-engage warte auf engage_ready (block: {blocked})")

                elif phase == Phase.ENGAGE:
                    ready = values.get("state.engage_ready") == "true"
                    allowed = values.get("lane_keeper.engage_allowed") == "true"
                    can_engage = allowed if args.lane_only else ready
                    if elapsed - last_phase_print > 2.0:
                        gate = (
                            f"engage_allowed={allowed}"
                            if args.lane_only
                            else f"engage_ready={ready}"
                        )
                        print_status(
                            phase,
                            f"ENTER = engage senden | {gate} | warte auf Active...",
                            values,
                        )
                        last_phase_print = elapsed
                    if not engage_sent and args.auto_engage and can_engage:
                        request_engage(ws, args.lane_only)
                        engage_sent = True
                        print("  -> engage gesendet")
                    # Non-blocking check for Enter (Windows)
                    try:
                        import msvcrt

                        if msvcrt.kbhit():
                            ch = msvcrt.getwch()
                            if ch in ("\r", "\n"):
                                if can_engage:
                                    request_engage(ws, args.lane_only)
                                    engage_sent = True
                                    print("  -> engage gesendet (ENTER)")
                                elif args.lane_only:
                                    print(
                                        "  engage_allowed=false — "
                                        f"dist={values.get('lane_keeper.engage_dist_m', '?')} m"
                                    )
                                else:
                                    print(
                                        f"  engage_ready=false — blockiert: "
                                        f"{values.get('state.engage_blocked_by', '?')}"
                                    )
                    except ImportError:
                        pass
                    if state == "Active":
                        print("\n  *** ACTIVE erkannt — Logging startet ***\n")
                        phase = Phase.LOGGING
                        log_started = time.monotonic()
                    elif state == "Engaging":
                        print("  ... Engaging ...")

                elif phase == Phase.LOGGING:
                    if log_started is None:
                        log_started = time.monotonic()
                    log_elapsed = time.monotonic() - log_started
                    iso = datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")
                    row_vals = {k: values.get(k, "") for k in LOG_KEYS}
                    writer.writerow([iso, f"{elapsed:.3f}", "logging"] + [row_vals[k] for k in LOG_KEYS])
                    fh.flush()

                    if state == "Active":
                        active_rows += 1
                        lat = fnum(row_vals.get("lane_keeper.truck_lat_vs_centerline_m"))
                        seg0 = fnum(row_vals.get("lane_keeper.diag_truck_to_segp0_m"))
                        tseq = row_vals.get("lane_keeper.tick_seq")
                        if tseq and tseq.isdigit():
                            active_tick_seqs.append(int(tseq))
                        if lat is not None:
                            active_lat.append(lat)
                        if seg0 is not None:
                            active_segp0.append(seg0)
                        print(fmt_log_line(row_vals, log_elapsed))
                    else:
                        print(
                            f"  [!] Nicht mehr Active ({state}) — "
                            f"weiter fahren / erneut engage?"
                        )
                        if state in ("Off", "Fault", "Paused"):
                            phase = Phase.DONE
                            break

                    if args.log_sec > 0 and log_elapsed >= args.log_sec:
                        print(f"\n  {args.log_sec:.0f} s Active-Log fertig.")
                        phase = Phase.DONE

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n--- Strg+C ---")

    try:
        ws.close()
    except Exception:
        pass

    print(f"\nCSV: {out_path}")
    disengage_keys = {
        k: last_values.get(k, "")
        for k in (
            "state.last_transition",
            "state.last_disengage_reason",
            "state.state_age_ticks",
            "arbitration.steering_winner_plugin",
            "arbitration.steering_value",
            "scs_sdk_output.active",
            "scs_sdk_output.steer_written",
            "vjoy.last_raw_x",
            "speed_controller.throttle_cmd",
            "speed_controller.tick_seq",
        )
    }
    print_summary(active_lat, active_segp0, active_rows, active_tick_seqs, disengage_keys)
    copy_to_claude(out_path)
    return 0


def run_legacy(args: argparse.Namespace) -> int:
    """Ungefuhrter Modus (altes Verhalten)."""
    keys = LOG_KEYS
    interval = 1.0 / max(args.hz, 0.1)
    out_path = args.out or default_out_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ws = connect_ws(args.url)
    active_lat: list[float] = []
    active_segp0: list[float] = []
    active_tick_seqs: list[int] = []
    active_rows = 0
    total_rows = 0
    last_values: dict[str, str] = {}
    t0 = time.monotonic()
    deadline = t0 + args.seconds if args.seconds > 0 else None

    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["iso_time", "elapsed_s"] + keys)
        try:
            while True:
                if deadline and time.monotonic() >= deadline:
                    break
                values = fetch_keys(ws, keys)
                last_values = values
                elapsed = time.monotonic() - t0
                is_active = values.get("autopilot.state") == "Active"
                if not args.active_only or is_active:
                    iso = datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")
                    writer.writerow([iso, f"{elapsed:.3f}"] + [values.get(k, "") for k in keys])
                    fh.flush()
                    total_rows += 1
                    if is_active:
                        print(fmt_log_line(values, elapsed))
                if is_active:
                    active_rows += 1
                    tseq = values.get("lane_keeper.tick_seq")
                    if tseq and tseq.isdigit():
                        active_tick_seqs.append(int(tseq))
                    lat = fnum(values.get("lane_keeper.truck_lat_vs_centerline_m"))
                    seg0 = fnum(values.get("lane_keeper.diag_truck_to_segp0_m"))
                    if lat is not None:
                        active_lat.append(lat)
                    if seg0 is not None:
                        active_segp0.append(seg0)
                time.sleep(interval)
        except KeyboardInterrupt:
            pass

    disengage_keys = {
        k: last_values.get(k, "")
        for k in (
            "state.last_transition",
            "state.last_disengage_reason",
            "state.state_age_ticks",
        )
    }
    print_summary(active_lat, active_segp0, active_rows, active_tick_seqs, disengage_keys)
    copy_to_claude(out_path)
    return 0


def main() -> int:
    args = parse_args()
    if args.no_guided:
        return run_legacy(args)
    return run_guided(args)


if __name__ == "__main__":
    sys.exit(main())
