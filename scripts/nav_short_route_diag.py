#!/usr/bin/env python3
"""
nav_short_route_diag.py — R3: route_task-Kette bei KURZEN vs LANGEN Routen.

Reine Lese-Diagnose. Spiel/Route nicht anfassen.

Usage:
    python scripts/nav_short_route_diag.py --graph graph.json
    python scripts/nav_short_route_diag.py --graph graph.json --out outputs/2026-06-13/short-route-diag.txt
"""
from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path
from typing import IO, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_gps import OFF_GPS_TRIP_DIST, read_shm_nav_distance, resolve_gps_manager

try:
    import pymem
except ImportError:
    print("pip install pymem")
    sys.exit(1)

PROC = "eurotrucks2.exe"

# Bekannte lange-Route-Kette (1.59)
OFF_SRS = 0x08
OFF_A = 0x58
OFF_B = 0x2C0
OFF_RT_REF = 0x1A8
OFF_RT_BIAS = 0x18
OFF_PHYS_ITEMS = 0x50
ITEM_STRIDE = 0x40
OFF_ITEM_UID = 0x30
OFF_ITEM_ACTIVE = 0x0C
UID_MIN = 5_000_000_000_000_000_000

KNOWN_LONG_CHAIN = "gps+0x08 -> srs+0x58 -> A+0x2C0 -> B+0x1A8 -> route_task+0x18"


class Log:
    def __init__(self, path: Optional[Path] = None):
        self.path = path
        self.lines: list[str] = []
        self._fh: Optional[IO[str]] = None
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = open(path, "w", encoding="utf-8")

    def __call__(self, msg: str = "") -> None:
        print(msg)
        self.lines.append(msg)
        if self._fh:
            self._fh.write(msg + "\n")
            self._fh.flush()

    def close(self) -> None:
        if self._fh:
            self._fh.close()


def looks_like_ptr(v: int | None) -> bool:
    return v is not None and 0x10000 < v < 0x7FFFFFFFFFFF


def u64(pm, a: int) -> int | None:
    try:
        return pm.read_ulonglong(a)
    except Exception:
        return None


def u32(pm, a: int) -> int | None:
    try:
        return pm.read_uint(a)
    except Exception:
        return None


def f32(pm, a: int) -> float | None:
    try:
        return pm.read_float(a)
    except Exception:
        return None


def read_bytes(pm, a: int, n: int) -> bytes | None:
    try:
        return pm.read_bytes(a, n)
    except Exception:
        return None


def load_graph_uids(path: str) -> set[int]:
    import ijson

    uids: set[int] = set()
    with open(path, "rb") as f:
        for i, n in enumerate(ijson.items(f, "nodes.item")):
            uids.add(int(n["uid"]))
            if i > 800_000:
                break
    return uids


def dump_hex(log: Log, pm, base: int, size: int, label: str, highlight: set[int] | None = None) -> None:
    highlight = highlight or set()
    raw = read_bytes(pm, base, size)
    log(f"\n--- hex dump {label} @ 0x{base:X} ({size} bytes) ---")
    if not raw:
        log("  (unreadable)")
        return
    for off in range(0, len(raw), 8):
        if off + 8 > len(raw):
            break
        q = struct.unpack_from("<Q", raw, off)[0]
        lo = q & 0xFFFFFFFF
        hi = (q >> 32) & 0xFFFFFFFF
        mark = ""
        if off in highlight:
            mark = " <<<"
        if looks_like_ptr(q):
            mark += " [valid ptr]"
        log(f"  +0x{off:03X}  u64=0x{q:016X}  u32lo={lo:10d} u32hi={hi:10d}{mark}")


@dataclass
class RouteArrayScore:
    route_task: int
    arr_ptr: int
    count: int
    graph_hits: int
    first5_graph: int
    uids_first5: list[int]
    path_desc: str
    plausible_km: bool = False

    @property
    def quality(self) -> tuple[int, int, int]:
        return (self.first5_graph, self.graph_hits, self.count)


def read_uid_array(pm, arr_ptr: int, max_n: int = 4000) -> list[int]:
    uids: list[int] = []
    for i in range(max_n):
        raw = read_bytes(pm, arr_ptr + i * ITEM_STRIDE, ITEM_STRIDE)
        if not raw:
            break
        uid = struct.unpack_from("<Q", raw, OFF_ITEM_UID)[0]
        if uid == 0 or uid < UID_MIN:
            break
        uids.append(uid)
    while uids:
        raw = read_bytes(pm, arr_ptr + (len(uids) - 1) * ITEM_STRIDE, ITEM_STRIDE)
        if not raw:
            break
        if struct.unpack_from("<I", raw, OFF_ITEM_ACTIVE)[0] != 0:
            break
        uids.pop()
    return uids


def score_route_task(pm, rt: int, graph: set[int], path_desc: str, trip_m: float) -> RouteArrayScore | None:
    arr = u64(pm, rt + OFF_PHYS_ITEMS)
    if not looks_like_ptr(arr):
        return None
    uids = read_uid_array(pm, arr)
    if len(uids) < 3:
        return None
    first5_graph = sum(1 for u in uids[:5] if u in graph)
    if first5_graph < 3:
        return None
    graph_hits = sum(1 for u in uids if u in graph)
    # plausibel fuer ~3km: 15-80 items (weit gefasst)
    est_items = trip_m / 80.0 if trip_m > 0 else 40
    plausible = 10 <= len(uids) <= max(120, int(est_items * 3))
    return RouteArrayScore(
        route_task=rt,
        arr_ptr=arr,
        count=len(uids),
        graph_hits=graph_hits,
        first5_graph=first5_graph,
        uids_first5=uids[:5],
        path_desc=path_desc,
        plausible_km=plausible,
    )


def try_via_b_ref(pm, b: int, graph: set[int], path_desc: str, trip_m: float) -> RouteArrayScore | None:
    if not looks_like_ptr(b):
        return None
    ref = u64(pm, b + OFF_RT_REF)
    if not looks_like_ptr(ref):
        return None
    return score_route_task(pm, ref + OFF_RT_BIAS, graph, path_desc, trip_m)


def scan_base_for_route(pm, base: int, base_label: str, graph: set[int], trip_m: float,
                        scan_end: int = 0x400, step: int = 8) -> list[RouteArrayScore]:
    hits: list[RouteArrayScore] = []
    seen_rts: set[int] = set()

    def add(score: RouteArrayScore | None) -> None:
        if score and score.route_task not in seen_rts:
            seen_rts.add(score.route_task)
            hits.append(score)

    # Direkt: base koennte route_task sein
    add(score_route_task(pm, base, graph, f"{base_label} (direct route_task)", trip_m))

    # base koennte ref sein (route_task = base + 0x18)
    if looks_like_ptr(base - OFF_RT_BIAS):
        pass  # skip
    add(score_route_task(pm, base + OFF_RT_BIAS, graph, f"{base_label}+0x18 (as ref+0x18)", trip_m))

    for off in range(0, scan_end, step):
        p = u64(pm, base + off)
        if not looks_like_ptr(p):
            continue

        # Kind als route_task
        add(score_route_task(pm, p, graph, f"{base_label}+0x{off:X} -> * (direct rt)", trip_m))

        # Kind -> ref+0x18 via +0x1A8
        add(try_via_b_ref(pm, p, graph, f"{base_label}+0x{off:X} -> * -> +0x1A8", trip_m))

        # Kind+0x18 als route_task (wenn Kind=ref)
        add(score_route_task(pm, p + OFF_RT_BIAS, graph, f"{base_label}+0x{off:X} ptr+0x18", trip_m))

        # Zweite Ebene: p -> q -> route_task
        for off2 in range(0, min(0x200, scan_end), 8):
            q = u64(pm, p + off2)
            if not looks_like_ptr(q):
                continue
            add(score_route_task(pm, q, graph,
                                 f"{base_label}+0x{off:X}->+0x{off2:X} direct rt", trip_m))
            add(try_via_b_ref(pm, q, graph,
                              f"{base_label}+0x{off:X}->+0x{off2:X}->+0x1A8", trip_m))
            # A-typisch: p=srs, off2=0x58 -> A, dann B-Slots scannen
            if off2 == OFF_A:
                for b_off in range(0, 0x400, 8):
                    b = u64(pm, q + b_off)
                    if looks_like_ptr(b):
                        add(try_via_b_ref(pm, b, graph,
                                          f"{base_label}+0x{off:X}(A)+0x{b_off:X}->+0x1A8", trip_m))

    hits.sort(key=lambda s: s.quality, reverse=True)
    return hits


def step1_chain(log: Log, pm, gps: int, graph: set[int], trip_m: float) -> tuple[int | None, list[RouteArrayScore]]:
    log("=" * 72)
    log("SCHRITT 1: Bekannte Kette Stufe fuer Stufe")
    log("=" * 72)
    log(f"gps_manager     = 0x{gps:X}")
    log(f"trip_distance   = {trip_m:.1f} m (gps+0x21C)")
    shm = read_shm_nav_distance()
    if shm is not None:
        log(f"SHM nav_dist    = {shm:.1f} m  delta={abs(trip_m - shm):.2f}")

    srs = u64(pm, gps + OFF_SRS)
    log(f"\nsrs = *(gps+0x08) = 0x{srs:X}  valid={looks_like_ptr(srs)}")
    if looks_like_ptr(srs):
        dump_hex(log, pm, srs, 0x40, "srs header")
    else:
        log("[ABBRUCH] srs ungueltig")
        return None, []

    a = u64(pm, srs + OFF_A)
    log(f"\nA = *(srs+0x58) = 0x{a:X}  valid={looks_like_ptr(a)}")
    if looks_like_ptr(a):
        dump_hex(log, pm, a, 0x40, "A header")
    else:
        log("[ABBRUCH] A ungueltig")
        return None, []

    b_known = u64(pm, a + OFF_B)
    log(f"\nB = *(A+0x2C0) = 0x{b_known:X}  valid={looks_like_ptr(b_known)}")
    if not looks_like_ptr(b_known):
        log("  -> BEKANNTER PFAD BRICHT HIER AB (A+0x2C0 kein gueltiger Pointer)")

    # Fenster um +0x2C0
    win_start = 0x280
    win_size = 0x100
    log(f"\n--- Fenster A+0x{win_start:X}..+0x{win_start + win_size:X} (um +0x2C0) ---")
    dump_hex(log, pm, a + win_start, win_size, "A slot window", highlight={OFF_B - win_start})

    # Alle gueltigen Ptrs im Fenster testen
    alt_from_window: list[RouteArrayScore] = []
    raw = read_bytes(pm, a + win_start, win_size)
    if raw:
        log("\n  Ptr-Kandidaten im Fenster -> +0x1A8-Test:")
        for off in range(0, len(raw) - 7, 8):
            q = struct.unpack_from("<Q", raw, off)[0]
            abs_off = win_start + off
            if not looks_like_ptr(q):
                continue
            sc = try_via_b_ref(pm, q, graph, f"A+0x{abs_off:X} (->0x{q:X})->+0x1A8", trip_m)
            mark = ""
            if sc:
                mark = f" *** ROUTE ARRAY count={sc.count} graph={sc.graph_hits}"
                alt_from_window.append(sc)
            log(f"    A+0x{abs_off:03X} ptr=0x{q:X}{mark}")

    # Voller A-Scan nach alternativem B-Slot
    log("\n--- A+0x00..0x400: alle Ptr mit +0x1A8 -> route_task ---")
    a_hits = scan_base_for_route(pm, a, "A", graph, trip_m, scan_end=0x400)
    # Filter nur die die via +0x1A8 kommen (nicht direct rt auf A selbst)
    a_via_1a8 = [h for h in a_hits if "+0x1A8" in h.path_desc and "A+" in h.path_desc]
    for i, h in enumerate(a_via_1a8[:20]):
        known = " [KNOWN LONG +0x2C0]" if f"A+0x{OFF_B:X}" in h.path_desc else ""
        log(f"  #{i+1} {h.path_desc}{known}")
        log(f"       rt=0x{h.route_task:X} arr=0x{h.arr_ptr:X} count={h.count} "
            f"graph_hits={h.graph_hits} first5={h.uids_first5}")

    best = a_via_1a8[0] if a_via_1a8 else (alt_from_window[0] if alt_from_window else None)
    if looks_like_ptr(b_known):
        ref = u64(pm, b_known + OFF_RT_REF)
        if looks_like_ptr(ref):
            rt = ref + OFF_RT_BIAS
            log(f"\n[OK] Bekannte Kette durch: route_task=0x{rt:X}")
            sc = score_route_task(pm, rt, graph, KNOWN_LONG_CHAIN, trip_m)
            if sc:
                return rt, [sc] + a_hits
            return rt, a_hits

    return (best.route_task if best else None), a_via_1a8 + alt_from_window


def step2_gps_scan(log: Log, pm, gps: int, graph: set[int], trip_m: float) -> list[RouteArrayScore]:
    log("\n" + "=" * 72)
    log("SCHRITT 2: gps+0x00..0x400 Breitensuche nach route_task / items+0x50")
    log("=" * 72)
    hits = scan_base_for_route(pm, gps, "gps", graph, trip_m, scan_end=0x400)
    for i, h in enumerate(hits[:25]):
        log(f"  #{i+1} {h.path_desc}")
        log(f"       rt=0x{h.route_task:X} count={h.count} graph={h.graph_hits}/{h.count} "
            f"plausible_km={h.plausible_km} first5={h.uids_first5}")
    if not hits:
        log("  Kein Kandidat mit >=3/5 graph-Treffern in den ersten 5 UIDs.")
    return hits


def step3_verify(log: Log, pm, candidates: list[RouteArrayScore], graph: set[int], trip_m: float) -> None:
    log("\n" + "=" * 72)
    log("SCHRITT 3: Verifikation & Vergleich mit langer Route")
    log("=" * 72)
    log(f"Bekannte lange Route: {KNOWN_LONG_CHAIN}")
    log(f"Aktuelle trip_distance: {trip_m:.1f} m")

    if not candidates:
        log("\n[ERGEBNIS] KEIN gueltiger route_task-Pfad gefunden.")
        log("Kurze Routen nutzen vermutlich eine andere Speicherstruktur oder")
        log("der bekannte B-Slot (+0x2C0) ist nur bei langen Routen belegt.")
        log("Empfehlung: robustes Abfangen + eigenes Routing fuer kurze Routen.")
        return

    # Dedupe by route_task
    seen: set[int] = set()
    unique: list[RouteArrayScore] = []
    for c in candidates:
        if c.route_task not in seen:
            seen.add(c.route_task)
            unique.append(c)
    unique.sort(key=lambda s: (s.plausible_km, s.quality), reverse=True)

    log(f"\n{len(unique)} eindeutige Kandidaten:")
    for i, h in enumerate(unique[:10]):
        log(f"\n--- Kandidat #{i+1} ---")
        log(f"  Pfad:     {h.path_desc}")
        log(f"  route_task: 0x{h.route_task:X}")
        log(f"  arr+0x50:   0x{h.arr_ptr:X}")
        log(f"  count:      {h.count} (erwartet ~{int(trip_m/80)}..{int(trip_m/50)} fuer {trip_m/1000:.1f}km)")
        log(f"  graph:      {h.graph_hits}/{h.count} Treffer, first5_in_graph={h.first5_graph}/5")
        log(f"  plausible:  {h.plausible_km}")
        log(f"  UIDs[0:5]: {h.uids_first5}")
        # array_dyn block
        dump_hex(log, pm, h.route_task + OFF_PHYS_ITEMS - 0x10, 0x30,
                 f"route_task+0x40..0x68 (#{i+1})")

    best = unique[0]
    uses_known = f"A+0x{OFF_B:X}" in best.path_desc or KNOWN_LONG_CHAIN in best.path_desc
    log("\n" + "-" * 72)
    if uses_known:
        log("[ERGEBNIS] Gemeinsamer Pfad mit langer Route GEFUNDEN.")
    elif best.plausible_km and best.first5_graph >= 4:
        log("[ERGEBNIS] ALTERNATIVER Pfad fuer kurze Route GEFUNDEN (nicht +0x2C0).")
        log(f"  Bester Pfad: {best.path_desc}")
    else:
        log("[ERGEBNIS] Schwache Kandidaten — kein robuster gemeinsamer Pfad.")
    log(f"  Bester: count={best.count} graph={best.graph_hits} path={best.path_desc}")


def write_summary_md(log_path: Path, trip_m: float, candidates: list[RouteArrayScore]) -> Path:
    today = date.today().isoformat()
    md_path = Path(f"outputs/{today}/phase-r3-short-route-diag.md")
    claude_path = Path("outputs/claude/phase-r3-short-route-diag.md")
    md_path.parent.mkdir(parents=True, exist_ok=True)
    claude_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "---",
        "tags: [phase, truckpilot, nav, r3]",
        "---",
        "",
        "# R3 Kurz-Route route_task Diagnose",
        "",
        f"- **Datum:** {today}",
        f"- **trip_distance:** {trip_m:.1f} m",
        f"- **Log:** `{log_path}`",
        "",
        "## Bekannte lange Route",
        "",
        f"`{KNOWN_LONG_CHAIN}`",
        "",
        "## Ergebnis",
        "",
    ]
    if not candidates:
        lines += [
            "Kein gueltiger route_task-Pfad. A+0x2C0 bei kurzer Route ungueltig.",
            "Kurze Routen vermutlich andere Struktur → Fallback-Routing noetig.",
        ]
    else:
        best = candidates[0]
        lines += [
            f"| Feld | Wert |",
            f"|------|------|",
            f"| Pfad | `{best.path_desc}` |",
            f"| route_task | `0x{best.route_task:X}` |",
            f"| count | {best.count} |",
            f"| graph hits | {best.graph_hits}/{best.count} |",
            f"| plausible 3.3km | {best.plausible_km} |",
        ]
    text = "\n".join(lines) + "\n"
    md_path.write_text(text, encoding="utf-8")
    claude_path.write_text(text, encoding="utf-8")
    return md_path


def main() -> int:
    ap = argparse.ArgumentParser(description="R3 Kurz-Route route_task Diagnose")
    ap.add_argument("--graph", default="graph.json", help="graph.json Gold-Standard")
    ap.add_argument("--out", help="Log-Datei (default: outputs/YYYY-MM-DD/short-route-diag.txt)")
    ap.add_argument("--no-shm", action="store_true", help="SHM-Check bei gps resolve ueberspringen")
    args = ap.parse_args()

    today = date.today().isoformat()
    out_path = Path(args.out) if args.out else Path(f"outputs/{today}/short-route-diag.txt")
    log = Log(out_path)

    try:
        pm = pymem.Pymem(PROC)
    except pymem.exception.ProcessNotFound:
        log("[FEHLER] eurotrucks2.exe laeuft nicht.")
        return 1

    try:
        gps = resolve_gps_manager(pm, verify_shm=not args.no_shm)
    except RuntimeError as e:
        log(f"[FEHLER] gps_manager: {e}")
        return 1

    trip_m = f32(pm, gps + OFF_GPS_TRIP_DIST) or 0.0
    log(f"=== nav_short_route_diag ===")
    log(f"graph: {args.graph}")
    log(f"output: {out_path}")

    graph = load_graph_uids(args.graph)
    log(f"graph nodes loaded: {len(graph)}")

    _, step1_hits = step1_chain(log, pm, gps, graph, trip_m)
    step2_hits = step2_gps_scan(log, pm, gps, graph, trip_m)

    all_hits = step1_hits + step2_hits
    all_hits.sort(key=lambda s: (s.plausible_km, s.quality), reverse=True)
    step3_verify(log, pm, all_hits, graph, trip_m)

    md = write_summary_md(out_path, trip_m, all_hits)
    log(f"\nMarkdown: {md}")
    log(f"Kopie:    outputs/claude/phase-r3-short-route-diag.md")
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
