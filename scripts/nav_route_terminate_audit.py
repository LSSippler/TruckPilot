#!/usr/bin/env python3
"""
nav_route_terminate_audit.py — R3 Teil 1: echtes Routenende / Count-Feld finden.

Usage:
    python scripts/nav_route_terminate_audit.py --graph graph.json
    python scripts/nav_route_terminate_audit.py --graph graph.json --max-scan 2500
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import ijson

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_gps import resolve_gps_manager

try:
    import pymem
except ImportError:
    print("pip install pymem")
    sys.exit(1)

OFF_SIMPLE_ROUTE_SRC = 0x08
OFF_SRS_ROUTE_A = 0x58
OFF_ROUTE_B_SLOT = 0x2C0
OFF_ROUTE_TASK_REF = 0x1A8
OFF_ROUTE_TASK_BIAS = 0x18
OFF_PHYS_ITEMS = 0x50
ITEM_STRIDE = 0x40
OFF_ITEM_UID = 0x30
UID_MIN_HEUR = 5_000_000_000_000_000_000
MAX_ITEMS = 4000


def looks_like_ptr(v: int | None) -> bool:
    return v is not None and 0x10000 < v < 0x7FFFFFFFFFFF


def u64(pm, a):
    try:
        return pm.read_ulonglong(a)
    except Exception:
        return None


def u32(pm, a):
    try:
        return pm.read_uint(a)
    except Exception:
        return None


def read_bytes(pm, a, n):
    try:
        return pm.read_bytes(a, n)
    except Exception:
        return None


def resolve_route_task(pm, gps):
    """gps -> route_task with per-step validation. Returns (rt, err)."""
    if not looks_like_ptr(gps):
        return None, f"gps_manager ungueltig: 0x{gps:X}"

    srs = u64(pm, gps + OFF_SIMPLE_ROUTE_SRC)
    if not looks_like_ptr(srs):
        return None, "gps+0x08 simple_route_source null - keine aktive Route oder im Menue?"

    a = u64(pm, srs + OFF_SRS_ROUTE_A)
    if not looks_like_ptr(a):
        return None, f"srs+0x58 null (0x{a:X}) - Route-Kette unterbrochen"

    b = u64(pm, a + OFF_ROUTE_B_SLOT)
    if not looks_like_ptr(b):
        return None, f"A+0x2C0 null (0x{b:X}) - Route evtl. neu berechnet / nicht gesetzt"

    ref = u64(pm, b + OFF_ROUTE_TASK_REF)
    if not looks_like_ptr(ref):
        return None, f"B+0x1A8 route_task_ref null (0x{ref:X})"

    return ref + OFF_ROUTE_TASK_BIAS, None


def load_graph(path: str) -> set[int]:
    g = set()
    with open(path, "rb") as f:
        for i, n in enumerate(ijson.items(f, "nodes.item")):
            g.add(int(n["uid"]))
            if i > 800_000:
                break
    return g


def uid_heuristic_end(uids_raw: list[int]) -> int:
    """Index of first failing uid (5e18 heuristic)."""
    for i, uid in enumerate(uids_raw):
        if uid == 0 or uid < UID_MIN_HEUR:
            return i
    return len(uids_raw)


def graph_gold_end(uids_raw: list[int], graph: set[int]) -> int:
    """Last index whose uid is in graph.json; +1 = first past end."""
    last = -1
    for i, uid in enumerate(uids_raw):
        if uid in graph:
            last = i
        elif last >= 0:
            break
    return last + 1  # count of valid graph hits if contiguous from 0


def scan_count_fields(pm, route_task: int, gold_count: int, window: int = 0x200):
    print(f"\n=== Count-Feld-Scan route_task+0..0x{window:X} (gold={gold_count}) ===")
    hits_u64 = []
    hits_u32 = []
    for off in range(0, window, 4):
        v32 = u32(pm, route_task + off)
        if v32 is not None and 1 <= v32 <= MAX_ITEMS and v32 == gold_count:
            hits_u32.append(off)
        if off % 8 == 0:
            v64 = u64(pm, route_task + off)
            if v64 is not None and 1 <= v64 <= MAX_ITEMS and v64 == gold_count:
                hits_u64.append(off)
    print(f"  u32 == {gold_count}: {[hex(o) for o in hits_u32]}")
    print(f"  u64 == {gold_count}: {[hex(o) for o in hits_u64]}")

    # phys_items block detail
    base = OFF_PHYS_ITEMS
    print(f"\n=== array_dyn @ route_task+0x{base:X} (wide) ===")
    for off in range(base, base + 0x40, 8):
        v = u64(pm, route_task + off)
        v32a = u32(pm, route_task + off)
        v32b = u32(pm, route_task + off + 4)
        mark = ""
        if v == gold_count or v32a == gold_count or v32b == gold_count:
            mark = " *** GOLD"
        print(f"  +0x{off:03X}  u64=0x{v:X}  u32lo={v32a} u32hi={v32b}{mark}")


def dump_item_hex(pm, arr_ptr: int, idx: int) -> str:
    raw = read_bytes(pm, arr_ptr + idx * ITEM_STRIDE, ITEM_STRIDE)
    if not raw:
        return "<unreadable>"
    return raw.hex(" ")


def item_fields(pm, arr_ptr: int, idx: int) -> dict:
    base = arr_ptr + idx * ITEM_STRIDE
    raw = read_bytes(pm, base, ITEM_STRIDE) or b"\x00" * ITEM_STRIDE
    uid = struct.unpack_from("<Q", raw, 0x30)[0]
    fields = {"uid": uid}
    for off in range(0, ITEM_STRIDE, 8):
        fields[f"+0x{off:02X}"] = struct.unpack_from("<Q", raw, off)[0]
    for off in range(0, ITEM_STRIDE, 4):
        fields[f"+0x{off:02X}u32"] = struct.unpack_from("<I", raw, off)[0]
    return fields


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    ap.add_argument("--max-scan", type=int, default=MAX_ITEMS, help="max items to read for audit")
    args = ap.parse_args()

    try:
        pm = pymem.Pymem("eurotrucks2.exe")
    except pymem.exception.ProcessNotFound:
        print("[FEHLER] eurotrucks2.exe laeuft nicht.")
        sys.exit(1)

    try:
        gps = resolve_gps_manager(pm)
    except RuntimeError as e:
        print(f"[FEHLER] gps_manager: {e}")
        sys.exit(1)

    rt, err = resolve_route_task(pm, gps)
    if err:
        print(f"[FEHLER] {err}")
        print(f"gps_manager = 0x{gps:X}")
        print("Hinweis: Truck in der Welt mit aktiver Navigationsroute - nicht Hauptmenue/Pause.")
        sys.exit(1)

    arr = u64(pm, rt + OFF_PHYS_ITEMS)
    if not looks_like_ptr(arr):
        print(f"[FEHLER] route_task+0x50 items.ptr ungueltig: 0x{arr:X}")
        sys.exit(1)

    print(f"gps_manager  = 0x{gps:X}")
    print(f"route_task   = 0x{rt:X}")
    print(f"items.ptr    = 0x{arr:X}")

    graph = load_graph(args.graph)
    print(f"graph nodes  = {len(graph)}")

    # Read raw uids up to max_scan
    uids = []
    limit = min(args.max_scan, MAX_ITEMS)
    for i in range(limit):
        uid = u64(pm, arr + i * ITEM_STRIDE + OFF_ITEM_UID)
        if uid is None:
            break
        uids.append(uid)

    heur_end = uid_heuristic_end(uids)
    graph_count = graph_gold_end(uids, graph)

    # Also find first index not in graph (strict)
    first_miss = None
    graph_hits = 0
    for i, uid in enumerate(uids):
        if uid in graph:
            graph_hits += 1
        elif graph_hits > 0 and first_miss is None:
            first_miss = i
            break

    print(f"\n=== Gold-Standard graph.json ===")
    print(f"  contiguous graph hits from 0: {graph_count}")
    print(f"  total graph hits (scan):     {graph_hits}")
    print(f"  first index NOT in graph:    {first_miss}")
    print(f"  5e18 heuristic end index:    {heur_end}")
    print(f"  delta (graph - heuristic):   {graph_count - heur_end}")

    gold = graph_count if graph_count > 0 else graph_hits
    scan_count_fields(pm, rt, gold)

    # Also scan with heuristic count for comparison
    if heur_end != gold:
        print(f"\n=== Count scan for heuristic end ({heur_end}) ===")
        scan_count_fields(pm, rt, heur_end)

    # Item layout dump around boundary
    boundary = gold if gold > 0 else heur_end
    print(f"\n=== Item dump around boundary (gold={boundary}) ===")
    for i in range(max(0, boundary - 3), boundary + 3):
        f = item_fields(pm, arr, i)
        in_g = f["uid"] in graph
        tag = "GRAPH" if in_g else "miss"
        print(f"  item[{i:4d}] uid={f['uid']} [{tag}]")
        print(f"           hex: {dump_item_hex(pm, arr, i)}")

    # Look for sentinel pattern: compare last valid vs first invalid
    if boundary > 0 and boundary < len(uids):
        print(f"\n=== Diff last-valid vs first-past (idx {boundary-1} vs {boundary}) ===")
        a = read_bytes(pm, arr + (boundary - 1) * ITEM_STRIDE, ITEM_STRIDE) or b""
        b = read_bytes(pm, arr + boundary * ITEM_STRIDE, ITEM_STRIDE) or b""
        for off in range(0, ITEM_STRIDE, 4):
            va = struct.unpack_from("<I", a, off)[0] if len(a) >= off + 4 else 0
            vb = struct.unpack_from("<I", b, off)[0] if len(b) >= off + 4 else 0
            if va != vb:
                print(f"  +0x{off:02X}: 0x{va:08X} -> 0x{vb:08X}")

    # Scan route_task for count in wider range including srs parent
    print(f"\n=== Quick u32 scan 0..0x100 step 4 (values 1..2500) ===")
    for off in range(0, 0x100, 4):
        v = u32(pm, rt + off)
        if v is not None and 1 <= v <= 2500:
            mark = " ***" if v == gold else ""
            print(f"  rt+0x{off:03X} = {v}{mark}")


if __name__ == "__main__":
    main()
