#!/usr/bin/env python3
"""
nav_ptr_layout_audit.py — R3 Nachtrag: route_task+0x50/+0x58 Layout klären.

Usage:
    python scripts/nav_ptr_layout_audit.py
    python scripts/nav_ptr_layout_audit.py --graph graph.json
"""
from __future__ import annotations

import argparse
import struct
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_gps import resolve_gps_manager

try:
    import pymem
except ImportError:
    print("pip install pymem")
    sys.exit(1)

OFF_PHYS_ITEMS = 0x50
OFF_PHYS_ITEMS_B = 0x58
OFF_PHYS_ITEMS_C = 0x60
ITEM_STRIDE = 0x40
OFF_ITEM_UID = 0x30
OFF_ITEM_ACTIVE = 0x0C
UID_MIN = 5_000_000_000_000_000_000


def resolve_route_task(pm, gps):
    """gps -> route_task with per-step validation. Returns (rt, err)."""
    if not looks_like_ptr(gps):
        return None, f"gps_manager ungueltig: 0x{gps:X}"

    srs = u64(pm, gps + 0x08)
    if not looks_like_ptr(srs):
        return None, "gps+0x08 simple_route_source null - keine aktive Route oder im Menue?"

    a = u64(pm, srs + 0x58)
    if not looks_like_ptr(a):
        return None, f"srs+0x58 null (0x{a:X}) - Route-Kette unterbrochen"

    b = u64(pm, a + 0x2C0)
    if not looks_like_ptr(b):
        return None, f"A+0x2C0 null (0x{b:X}) - Route evtl. neu berechnet / nicht gesetzt"

    ref = u64(pm, b + 0x1A8)
    if not looks_like_ptr(ref):
        return None, f"B+0x1A8 route_task_ref null (0x{ref:X})"

    return ref + 0x18, None


def chain(pm, gps):
    """Legacy alias — raises SystemExit on failure."""
    rt, err = resolve_route_task(pm, gps)
    if err:
        print(f"[FEHLER] {err}")
        print("Hinweis: ETS2 mit gesetzter Route in der Welt starten, nicht Hauptmenue.")
        print("  python scripts/read_shm_nav.py")
        print("  python scripts/resolve_gps.py")
        sys.exit(1)
    return rt


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


def looks_like_ptr(v):
    return v is not None and 0x10000 < v < 0x7FFFFFFFFFFF


def walk_5e18_trim(pm, arr, max_n=2500):
    uids = []
    for i in range(max_n):
        raw = read_bytes(pm, arr + i * ITEM_STRIDE, ITEM_STRIDE)
        if not raw:
            break
        uid = struct.unpack_from("<Q", raw, OFF_ITEM_UID)[0]
        if uid == 0 or uid < UID_MIN:
            break
        uids.append(uid)
    flags = []
    for i in range(len(uids)):
        raw = read_bytes(pm, arr + i * ITEM_STRIDE, ITEM_STRIDE)
        flags.append(struct.unpack_from("<I", raw, OFF_ITEM_ACTIVE)[0])
    while uids and flags[-1] == 0:
        uids.pop()
        flags.pop()
    return uids


def graph_count(uids, graph):
    if not graph:
        return None, None
    last = max((i for i, u in enumerate(uids) if u in graph), default=-1)
    hits = sum(1 for u in uids if u in graph)
    return last + 1, hits


def ptr_delta_counts(ptr_a, ptr_b, count, label):
    if not ptr_a or not ptr_b:
        print(f"  {label}: invalid ptrs")
        return
    diff = ptr_b - ptr_a
    print(f"  {label}: ptr_b - ptr_a = 0x{diff:X} ({diff} bytes)")
    for stride in (0x40, 0x20, 0x10, 8):
        if stride:
            slots = diff / stride
            mark = " *** MATCH" if abs(slots - count) < 0.01 else ""
            print(f"    / 0x{stride:X} ({stride}) = {slots:.2f}{mark}")


def dump_header(pm, ptr, name):
    print(f"\n--- dump 0x40 @ {name} (0x{ptr:X}) ---")
    raw = read_bytes(pm, ptr, 0x40)
    if not raw:
        print("  unreadable")
        return
    for off in range(0, 0x40, 8):
        q = struct.unpack_from("<Q", raw, off)[0]
        lo = q & 0xFFFFFFFF
        hi = (q >> 32) & 0xFFFFFFFF
        print(f"  +0x{off:02X}  u64=0x{q:016X}  u32lo={lo} u32hi={hi}")


def compare_arrays(pm, ptr50, ptr58, n=8):
    print(f"\n--- first {n} UIDs: ptr50 vs ptr58 (same indices?) ---")
    for i in range(n):
        u50 = u64(pm, ptr50 + i * ITEM_STRIDE + OFF_ITEM_UID)
        u58 = u64(pm, ptr58 + i * ITEM_STRIDE + OFF_ITEM_UID) if ptr58 else None
        same = u50 == u58 if u58 is not None else False
        print(f"  [{i}] ptr50 uid={u50}  ptr58 uid={u58}  same={same}")


def scan_o0c_stats(pm, arr, n_items):
    zeros = []
    nonzeros = []
    for i in range(n_items):
        raw = read_bytes(pm, arr + i * ITEM_STRIDE, ITEM_STRIDE)
        if not raw:
            break
        v = struct.unpack_from("<I", raw, OFF_ITEM_ACTIVE)[0]
        if v == 0:
            zeros.append(i)
        else:
            nonzeros.append(v)
    print(f"\n=== +0x0C stats over {n_items} scanned items (5e18 prefix) ===")
    print(f"  zero count: {len(zeros)} / {n_items}")
    print(f"  first 15 zero indices: {zeros[:15]}")
    print(f"  last 5 zero indices: {zeros[-5:] if zeros else []}")
    if nonzeros:
        ctr = Counter(nonzeros)
        print(f"  top nonzero values: {ctr.most_common(8)}")


def chunk_hypothesis(pm, arr, chunk_size=202):
    print(f"\n=== Chunk hypothesis (size={chunk_size}) ===")
    # check if uid at index k*chunk repeats pattern or ptr jumps
    for k in range(0, 6):
        i = k * chunk_size
        uid = u64(pm, arr + i * ITEM_STRIDE + OFF_ITEM_UID)
        o0c = u32(pm, arr + i * ITEM_STRIDE + OFF_ITEM_ACTIVE)
        print(f"  chunk {k} start idx {i}: uid={uid} +0x0C=0x{o0c or 0:X}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", help="graph.json for gold count")
    args = ap.parse_args()

    graph = None
    if args.graph:
        import ijson

        graph = set()
        with open(args.graph, "rb") as f:
            for i, n in enumerate(ijson.items(f, "nodes.item")):
                graph.add(int(n["uid"]))
                if i > 800_000:
                    break

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

    ptr50 = u64(pm, rt + OFF_PHYS_ITEMS)
    if not looks_like_ptr(ptr50):
        print(f"[FEHLER] route_task+0x50 items.ptr ungueltig: 0x{ptr50:X}")
        sys.exit(1)
    ptr58 = u64(pm, rt + OFF_PHYS_ITEMS_B)
    ptr60 = u64(pm, rt + OFF_PHYS_ITEMS_C)

    uids = walk_5e18_trim(pm, ptr50)
    count = len(uids)
    g_count, g_hits = graph_count(uids, graph) if graph else (None, None)

    print("=== Session ===")
    print(f"gps_manager   0x{gps:X}")
    print(f"route_task    0x{rt:X}")
    print(f"+0x50 ptr     0x{ptr50:X}")
    print(f"+0x58 ptr     0x{ptr58:X}")
    print(f"+0x60 ptr     0x{ptr60:X}")
    print(f"trim walk count = {count}")
    if graph:
        print(f"graph gold (last hit+1) = {g_count}, total hits in walk = {g_hits}")

    route_type = "LONG (~1085)" if count > 200 else "SHORT (~37)" if count < 80 else f"MEDIUM ({count})"
    print(f"route class: {route_type}")

    print("\n=== Task 1: pointer deltas vs count ===")
    for label, pa, pb in [
        ("+0x58 - +0x50", ptr50, ptr58),
        ("+0x60 - +0x50", ptr50, ptr60),
        ("+0x58 - +0x60", ptr60, ptr58),
    ]:
        if looks_like_ptr(pa) and looks_like_ptr(pb):
            ptr_delta_counts(pa, pb, count, label)

    print("\n=== route_task array_dyn block +0x50..+0x68 ===")
    for off in range(OFF_PHYS_ITEMS, OFF_PHYS_ITEMS + 0x20, 8):
        v = u64(pm, rt + off)
        lo = v & 0xFFFFFFFF if v else 0
        hi = (v >> 32) & 0xFFFFFFFF if v else 0
        print(f"  rt+0x{off:03X}  u64=0x{v:X}  u32lo={lo} u32hi={hi}")

    dump_header(pm, ptr50, "ptr50")
    if looks_like_ptr(ptr58):
        dump_header(pm, ptr58, "ptr58")
    if looks_like_ptr(ptr60):
        dump_header(pm, ptr60, "ptr60")

    compare_arrays(pm, ptr50, ptr58)

    # ptr58 == ptr50 + 202 * stride (same backing array, not separate)
    if looks_like_ptr(ptr50) and looks_like_ptr(ptr58):
        off202 = ptr50 + 202 * ITEM_STRIDE
        print(f"\n=== ptr58 identity ===")
        print(f"  ptr50 + 202*0x40 = 0x{off202:X}")
        print(f"  ptr58            = 0x{ptr58:X}")
        print(f"  same address: {off202 == ptr58}")
        if off202 == ptr58:
            u50_202 = u64(pm, ptr50 + 202 * ITEM_STRIDE + OFF_ITEM_UID)
            u58_0 = u64(pm, ptr58 + OFF_ITEM_UID)
            print(f"  ptr50[202] uid == ptr58[0] uid: {u50_202 == u58_0} ({u58_0})")

    # Is ptr58 = end of route?
    for stride in (0x40, 0x20, 0x10):
        end_calc = ptr50 + count * stride
        print(f"\n  ptr50 + count*{stride:#x} = 0x{end_calc:X}  (ptr58=0x{ptr58:X} match={end_calc == ptr58})")

    # scan prefix of ptr58 as item array
    if looks_like_ptr(ptr58):
        print("\n--- ptr58 as item array: UIDs [0..4] ---")
        for i in range(5):
            uid = u64(pm, ptr58 + i * ITEM_STRIDE + OFF_ITEM_UID)
            print(f"  ptr58[{i}] uid={uid}")

    scan_o0c_stats(pm, ptr50, min(count + 5, 1200))

    chunk_hypothesis(pm, ptr50, 202)
    if count > 202:
        # boundary around item 202
        for idx in [200, 201, 202, 203, 204]:
            raw = read_bytes(pm, ptr50 + idx * ITEM_STRIDE, ITEM_STRIDE)
            uid = struct.unpack_from("<Q", raw, OFF_ITEM_UID)[0]
            o0c = struct.unpack_from("<I", raw, OFF_ITEM_ACTIVE)[0]
            print(f"  idx {idx}: uid={uid} +0x0C=0x{o0c:08X}")

    # Tail vs mid +0x0C==0
    print("\n=== +0x0C==0 mid-route samples (in trim walk) ===")
    for i in range(min(count, 1200)):
        o0c = u32(pm, ptr50 + i * ITEM_STRIDE + OFF_ITEM_ACTIVE)
        if o0c == 0:
            uid = u64(pm, ptr50 + i * ITEM_STRIDE + OFF_ITEM_UID)
            in_g = uid in graph if graph else "?"
            tail = i >= count - 1
            print(f"  idx={i:4d} tail={tail} in_graph={in_g} uid={uid}")


if __name__ == "__main__":
    main()
