#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
truckpilot_nav_read.py
======================
Liest die ETS2-Navigationsroute (UID-Sequenz) aus dem Spielspeicher.
Diagnose-Werkzeug und Bruecke zum spaeteren Rust-Plugin.

Ziel-Spiel: ETS2 1.59.1.3s
Offsets sind 1.58-Erwartungswerte und MUESSEN fuer 1.59 verifiziert werden.

Voraussetzungen:
    pip install pymem
    ETS2 laeuft, Route gesetzt, Truck in der Welt.
    Skript als Administrator ausfuehren (gleiche Rechte wie das Spiel).

Nutzung:
    # Modus 1: ab bekannter gps_manager-Basis die Kette ablaufen
    python truckpilot_nav_read.py --gps 0x1DB2AD5F7B0

    # Modus 2: Struktur-Dump einer Adresse (Diagnose)
    python truckpilot_nav_read.py --probe 0x1DB2AD5F7B0

    # Modus 3: AOB-Scan-Versuch (1.58-Signatur, matcht in 1.59 evtl. nicht)
    python truckpilot_nav_read.py --aob

    # UID-Sequenz gegen graph.json pruefen
    python truckpilot_nav_read.py --gps 0x... --graph C:\\Users\\Sippler\\Documents\\TruckPilot\\graph.json
"""

import argparse
import struct
import sys
import ctypes
from ctypes import wintypes
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from resolve_gps import resolve_gps_manager

try:
    import pymem
    import pymem.process
except ImportError:
    print("pymem fehlt. Installieren mit:  pip install pymem")
    sys.exit(1)

PROC = "eurotrucks2.exe"

# Windows memory query constants
MEM_COMMIT = 0x1000
PAGE_GUARD = 0x100
PAGE_NOACCESS = 0x01
PAGE_READONLY = 0x02
PAGE_READWRITE = 0x04
PAGE_WRITECOPY = 0x08
PAGE_EXECUTE_READ = 0x20
PAGE_EXECUTE_READWRITE = 0x40
READABLE = {
    PAGE_READONLY, PAGE_READWRITE, PAGE_WRITECOPY,
    PAGE_EXECUTE_READ, PAGE_EXECUTE_READWRITE,
}


class MEMORY_BASIC_INFORMATION(ctypes.Structure):
    _fields_ = [
        ("BaseAddress", ctypes.c_void_p),
        ("AllocationBase", ctypes.c_void_p),
        ("AllocationProtect", wintypes.DWORD),
        ("RegionSize", ctypes.c_size_t),
        ("State", wintypes.DWORD),
        ("Protect", wintypes.DWORD),
        ("Type", wintypes.DWORD),
    ]

# --- Offsets (1.59 verifiziert) -----------------------------------------------
OFF_SIMPLE_ROUTE_SRC = 0x08    # gps_manager + 0x08 (POINTER in 1.59!)
OFF_SRS_ROUTE_A      = 0x58    # simple_route_source + 0x58
OFF_ROUTE_B_SLOT     = 0x2C0   # zw.B + 0x2C0
OFF_ROUTE_TASK_REF   = 0x1A8   # zw.C + 0x1A8 (route_task - 0x18)
OFF_ROUTE_TASK_BIAS  = 0x18    # route_task = ref + 0x18
OFF_ROUTE_TASK       = 0x20    # 1.58 only (deprecated)
OFF_PHYS_ITEMS       = 0x50    # route_task + 0x50 (array_dyn.ptr)
ITEM_STRIDE          = 0x40    # 1.59: 0x40 (1.58 war 0x20) — VERIFIZIERT
OFF_ITEM_NODE        = 0x00    # 1.59: UID embedded @ +0x30, kein node*-Pfad
OFF_ITEM_UID         = 0x30    # uid (u64) direkt im Item
OFF_ITEM_DIST_LEFT   = 0x14    # 1.59: UNVERIFIZIERT (keine m-Werte)
OFF_NODE_UID         = 0x30    # alias: embedded uid im Item
OFF_GPS_TRIP_DIST    = 0x21C   # gps_manager trip_distance (float, m)
OFF_GPS_TRIP_TIME    = 0x220   # gps_manager trip_time (float, s)

# 1.58 gps_manager-Signatur (lea rsi,[rdi+disp32]; xorps xmm1,xmm1)
AOB_GPS = "48 8D B7 ?? ?? ?? ?? 0F 57 C9"


def looks_like_ptr(v):
    return v is not None and 0x10000 < v < 0x7FFFFFFFFFFF


class Reader:
    def __init__(self):
        self.pm = pymem.Pymem(PROC)
        self.base = pymem.process.module_from_name(
            self.pm.process_handle, PROC).lpBaseOfDll
        self.module = pymem.process.module_from_name(
            self.pm.process_handle, PROC)
        print(f"[+] attached {PROC} base=0x{self.base:X} "
              f"size=0x{self.module.SizeOfImage:X}")

    def u64(self, addr):
        try:
            return self.pm.read_ulonglong(addr)
        except Exception:
            return None

    def u32(self, addr):
        try:
            return self.pm.read_uint(addr)
        except Exception:
            return None

    def f32(self, addr):
        try:
            return self.pm.read_float(addr)
        except Exception:
            return None

    # ---- Diagnose: Struktur-Dump --------------------------------------
    def probe(self, base):
        print(f"[probe] Struktur ab 0x{base:X}")
        for off in range(0, 0x68, 0x08):
            q = self.u64(base + off)
            f = self.f32(base + off)
            tag = " <ptr?>" if looks_like_ptr(q) else ""
            qs = f"0x{q:X}" if q is not None else "nil"
            print(f"  +0x{off:03X}  q={qs:<18}  f={f}{tag}")
        td = self.f32(base + OFF_GPS_TRIP_DIST)
        tt = self.f32(base + OFF_GPS_TRIP_TIME)
        print(f"  +0x21C trip_distance? = {td}")
        print(f"  +0x220 trip_time?     = {tt}")

    # ---- AOB-Scan ------------------------------------------------------
    def aob_scan(self):
        print(f"[aob] Scanne {PROC} nach 1.58-Signatur: {AOB_GPS}")
        addr = pymem.pattern.pattern_scan_module(
            self.pm.process_handle, self.module, AOB_GPS.encode())
        if not addr:
            # alternativ ueber rohes Pattern
            print("[aob] Kein Treffer im Modul. 1.59 hat vermutlich andere Bytes.")
            print("[aob] -> ueber Wert-Scan + 'find what accesses' eine 1.59-")
            print("        Signatur ableiten, dann AOB_GPS hier ersetzen.")
            return None
        disp = self.u32(addr + 3)
        gps_off = disp
        print(f"[aob] Treffer @0x{addr:X}, disp32=0x{gps_off:X}")
        print("[aob] disp32 ist der Feld-Offset, braucht noch game_ctrl-Basis.")
        return addr

    # ---- Float-Scan (Restdistanz / trip_distance) ----------------------
    def scan_float_between(self, lo, hi, max_hits=50):
        """Scan committed readable memory for f32 in [lo, hi]."""
        print(f"[scan] Float zwischen {lo:.1f} und {hi:.1f} ...")
        kernel32 = ctypes.windll.kernel32
        mbi = MEMORY_BASIC_INFORMATION()
        addr = 0
        hits = []
        regions = 0
        while addr < 0x7FFFFFFFFFFF and len(hits) < max_hits:
            ret = kernel32.VirtualQueryEx(
                self.pm.process_handle, ctypes.c_void_p(addr),
                ctypes.byref(mbi), ctypes.sizeof(mbi),
            )
            if ret == 0:
                break
            base = mbi.BaseAddress or 0
            size = mbi.RegionSize or 0
            next_addr = base + size
            if next_addr <= addr:
                break
            addr = next_addr
            regions += 1
            if mbi.State != MEM_COMMIT or mbi.Protect not in READABLE:
                continue
            if mbi.Protect & PAGE_GUARD:
                continue
            try:
                chunk = self.pm.read_bytes(base, min(size, 8 * 1024 * 1024))
            except Exception:
                continue
            for off in range(0, len(chunk) - 3, 4):
                v = struct.unpack_from("<f", chunk, off)[0]
                if lo <= v <= hi:
                    hits.append((base + off, v))
                    if len(hits) >= max_hits:
                        break
        print(f"[scan] {regions} Regionen, {len(hits)} Treffer (cap {max_hits})")
        for i, (a, v) in enumerate(hits[:30]):
            gps_cand = a - OFF_GPS_TRIP_DIST
            ttime = self.f32(a + 4)
            print(f"  {i:2d}  0x{a:X}  dist={v:.1f}  +4s={ttime}  "
                  f"(gps_base +0x21C: 0x{gps_cand:X})")
        return hits

    def scan_dist_time_pair(self, dist_m, time_s, dist_tol=3000, time_tol=5000):
        """Finde f32-Paare (distance, time) — typisch +0x21C/+0x220 in gps_manager."""
        lo, hi = dist_m - dist_tol, dist_m + dist_tol
        tlo, thi = time_s - time_tol, time_s + time_tol
        print(f"[pair] dist {lo:.0f}..{hi:.0f}, time {tlo:.0f}..{thi:.0f}")
        hits = self.scan_float_between(lo, hi, max_hits=200)
        pairs = []
        for a, v in hits:
            t = self.f32(a + 4)
            if t is not None and tlo <= t <= thi:
                base = a - OFF_GPS_TRIP_DIST
                pairs.append((base, v, t, a))
        print(f"[pair] {len(pairs)} Paare mit plausiblem time@+4")
        for i, (base, v, t, a) in enumerate(pairs[:20]):
            print(f"  {i:2d}  gps_base=0x{base:X}  dist@+21C={v:.1f}  time@+220={t:.1f}")
        return pairs

    # ---- Offset-Sweep gegen graph.json --------------------------------
    def scan_u64(self, value, max_hits=30):
        """Scan memory for exact u64 (e.g. graph node UID)."""
        print(f"[uid-scan] Suche u64 {value} (0x{value:X}) ...")
        kernel32 = ctypes.windll.kernel32
        mbi = MEMORY_BASIC_INFORMATION()
        addr = 0
        hits = []
        while addr < 0x7FFFFFFFFFFF and len(hits) < max_hits:
            ret = kernel32.VirtualQueryEx(
                self.pm.process_handle, ctypes.c_void_p(addr),
                ctypes.byref(mbi), ctypes.sizeof(mbi),
            )
            if ret == 0:
                break
            base = mbi.BaseAddress or 0
            size = mbi.RegionSize or 0
            next_addr = base + size
            if next_addr <= addr:
                break
            addr = next_addr
            if mbi.State != MEM_COMMIT or mbi.Protect not in READABLE:
                continue
            if mbi.Protect & PAGE_GUARD:
                continue
            needle = struct.pack("<Q", value)
            try:
                chunk = self.pm.read_bytes(base, min(size, 16 * 1024 * 1024))
            except Exception:
                continue
            start = 0
            while True:
                idx = chunk.find(needle, start)
                if idx < 0:
                    break
                hits.append(base + idx)
                if len(hits) >= max_hits:
                    break
                start = idx + 8
        print(f"[uid-scan] {len(hits)} Treffer")
        for i, a in enumerate(hits):
            # test ob UID an node+0x30 liegt
            node_base = a - OFF_NODE_UID
            print(f"  {i:2d}  0x{a:X}  (node+0x30 -> node=0x{node_base:X})")
        return hits

    def sweep_offsets(self, gps, graph_path):
        """Test route_task/phys_items/node_uid Kombinationen."""
        import json
        print(f"[sweep] gps_manager=0x{gps:X}, graph={graph_path}")
        with open(graph_path, "r", encoding="utf-8") as fh:
            g = json.load(fh)
        index = {n["uid"]: n for n in g["nodes"]}

        route_task_offs = [0x18, 0x20, 0x28]
        phys_offs = [0x48, 0x50, 0x58]
        uid_offs = [0x28, 0x30, 0x38]
        best = []

        for rt_off in route_task_offs:
            route_task = self.u64(gps + OFF_SIMPLE_ROUTE_SRC + rt_off)
            if not looks_like_ptr(route_task):
                continue
            for po in phys_offs:
                arr_ptr = self.u64(route_task + po)
                arr_size = self.u64(route_task + po + 8)
                if not looks_like_ptr(arr_ptr) or not arr_size or arr_size > 6000:
                    continue
                for uo in uid_offs:
                    hits = 0
                    first_uid = None
                    for i in range(min(arr_size, 10)):
                        item = arr_ptr + i * ITEM_STRIDE
                        node = self.u64(item + OFF_ITEM_NODE)
                        if not looks_like_ptr(node):
                            continue
                        uid = self.u64(node + uo)
                        if uid and uid in index:
                            hits += 1
                            if first_uid is None:
                                first_uid = uid
                    if hits > 0:
                        n = index[first_uid]
                        best.append((hits, rt_off, po, uo, first_uid, n["x"], n["z"]))
                        print(f"  rt=+0x{rt_off:X} phys=+0x{po:X} uid=+0x{uo:X} "
                              f"-> {hits}/10 graph hits, first uid={first_uid} "
                              f"({n['x']:.0f},{n['z']:.0f})")
        if not best:
            print("[sweep] Keine Kombination mit graph-Treffern.")
        else:
            best.sort(reverse=True)
            print(f"[sweep] BEST: route_task=+0x{best[0][1]:X} "
                  f"phys_items=+0x{best[0][2]:X} node_uid=+0x{best[0][3]:X}")
        return best

    # ---- Walk: Kette ablaufen, UIDs sammeln ---------------------------
    def resolve_route_task(self, gps):
        """1.59-Kette: gps -> srs* -> +0x58 -> +0x2C0 -> +0x1A8 (+0x18)."""
        srs = self.u64(gps + OFF_SIMPLE_ROUTE_SRC)
        if not looks_like_ptr(srs):
            return None, "simple_route_source null"
        a = self.u64(srs + OFF_SRS_ROUTE_A)
        if not looks_like_ptr(a):
            return None, f"srs+0x{OFF_SRS_ROUTE_A:X} null"
        b = self.u64(a + OFF_ROUTE_B_SLOT)
        if not looks_like_ptr(b):
            return None, f"A+0x{OFF_ROUTE_B_SLOT:X} null"
        ref = self.u64(b + OFF_ROUTE_TASK_REF)
        if not looks_like_ptr(ref):
            return None, f"B+0x{OFF_ROUTE_TASK_REF:X} null"
        rt = ref + OFF_ROUTE_TASK_BIAS
        return rt, None

    def walk_route_task(self, route_task, count=None):
        """1.59: UID embedded @ item+0x30, stride 0x40."""
        print(f"[walk-rt] route_task = 0x{route_task:X}")
        arr_ptr = self.u64(route_task + OFF_PHYS_ITEMS)
        arr_size = self.u64(route_task + OFF_PHYS_ITEMS + 8)
        if count:
            arr_size = count
        print(f"[walk-rt] items ptr={hex(arr_ptr) if arr_ptr else None} size={arr_size}")
        if not looks_like_ptr(arr_ptr):
            return []
        if not arr_size or arr_size > 6000:
            # 1.59: array_dyn.size oft 0 — bis leerer Slot scannen
            arr_size = 4000
            print("[walk-rt] size unplausibel, scanne bis uid==0")
        uids = []
        n = min(arr_size, 4000)
        for i in range(n):
            item = arr_ptr + i * ITEM_STRIDE
            uid = self.u64(item + OFF_ITEM_UID)
            if not uid or uid < 5_000_000_000_000_000_000:
                if not count:
                    break
                continue
            dist = self.f32(item + OFF_ITEM_DIST_LEFT)
            if uid:
                uids.append((uid, dist))
                if i < 5 or i >= n - 2:
                    print(f"  item[{i}] uid={uid} dist_left={dist}")
        if uids:
            print("-" * 60)
            print(f"[walk-rt] ERSTE UID = {uids[0][0]}")
            print(f"[walk-rt] LETZTE UID = {uids[-1][0]}")
        return uids

    def walk(self, gps):
        print(f"[walk] gps_manager = 0x{gps:X}")
        td = self.f32(gps + OFF_GPS_TRIP_DIST)
        print(f"[walk] +0x21C trip_distance = {td} (sollte ~Routenlaenge in m)")

        route_task, err = self.resolve_route_task(gps)
        if err:
            print(f"[walk] 1.59-Kette fehlgeschlagen: {err}")
            return []
        print(f"[walk] route_task = 0x{route_task:X} (via srs+0x58 -> +0x2C0 -> +0x1A8 +0x18)")
        return self.walk_route_task(route_task)


def check_graph(uids, graph_path):
    import json
    print(f"[graph] Lade {graph_path} ...")
    with open(graph_path, "r", encoding="utf-8") as fh:
        g = json.load(fh)
    index = {n["uid"]: n for n in g["nodes"]}
    print(f"[graph] {len(index)} Nodes geladen.")
    hits = 0
    for uid, _ in uids[:20]:
        n = index.get(uid)
        if n:
            hits += 1
            print(f"  GEFUNDEN uid={uid} -> x={n['x']:.1f} z={n['z']:.1f}")
        else:
            print(f"  NOT FOUND uid={uid}")
    print(f"[graph] {hits}/{min(len(uids),20)} der ersten UIDs in graph.json gefunden.")
    if hits > 0:
        print("[graph] >0 Treffer = Kette bewiesen. Koordinaten gegen Truck-Pos pruefen.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gps", help="gps_manager Basis (hex). Ohne --gps: --resolve oder --graph nutzt AOB.")
    ap.add_argument("--resolve", action="store_true",
                    help="gps_manager per AOB aufloesen (scripts/resolve_gps.py)")
    ap.add_argument("--route-task", help="route_task direkt (1.59 bypass)")
    ap.add_argument("--count", type=int, help="Item-Anzahl bei --route-task")
    ap.add_argument("--probe", help="Struktur-Dump einer Adresse")
    ap.add_argument("--aob", action="store_true", help="AOB-Scan-Versuch (1.58-Sig)")
    ap.add_argument("--graph", help="Pfad zu graph.json fuer UID-Check")
    ap.add_argument("--scan-float", type=float, metavar="METERS",
                    help="Float-Scan um METERS (+/-2000m)")
    ap.add_argument("--scan-pair", nargs=2, type=float, metavar=("DIST", "TIME"),
                    help="Scan dist+time Paar (SHM-Werte)")
    ap.add_argument("--scan-uid", type=lambda x: int(x, 0),
                    help="u64-Scan (dezimal oder 0x...)")
    ap.add_argument("--sweep", action="store_true",
                    help="Offset-Kombinationen gegen graph.json testen")
    args = ap.parse_args()

    r = Reader()

    if args.scan_float is not None:
        m = args.scan_float
        r.scan_float_between(m - 2000, m + 2000)
        return
    if args.scan_pair is not None:
        r.scan_dist_time_pair(args.scan_pair[0], args.scan_pair[1])
        return
    if args.scan_uid is not None:
        r.scan_u64(args.scan_uid)
        return
    if args.probe:
        r.probe(int(args.probe, 16))
        return
    if args.aob:
        r.aob_scan()
        return
    if args.route_task:
        uids = r.walk_route_task(int(args.route_task, 16), args.count)
        if args.graph and uids:
            check_graph(uids, args.graph)
        return
    if args.gps or args.resolve or args.graph:
        if args.gps:
            try:
                gps = int(args.gps, 16)
            except ValueError:
                print(f"Ungueltige --gps Adresse: {args.gps!r}")
                print("Nutze: python scripts/resolve_gps.py  oder  --resolve --graph graph.json")
                sys.exit(1)
        else:
            gps = resolve_gps_manager(r.pm)
            print(f"[+] gps_manager (AOB) = 0x{gps:X}")
        if args.sweep and args.graph:
            r.sweep_offsets(gps, args.graph)
            return
        uids = r.walk(gps)
        if args.graph and uids:
            check_graph(uids, args.graph)
        return
    print("Nichts zu tun. --resolve --graph graph.json  oder  --gps 0x...  Siehe --help.")


if __name__ == "__main__":
    main()
