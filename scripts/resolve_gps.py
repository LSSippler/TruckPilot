#!/usr/bin/env python3
"""
resolve_gps.py — stabile gps_manager-Auffindung für ETS2 1.59 (AOB + RIP).

Kette:
  mod + AOB  ->  mov rcx, [rip+disp]  @ Singleton-Slot
  *slot       ->  game_ctrl (Basis-Objekt)
  game_ctrl + 0x3E30  ->  gps_manager (eingebettet, kein separater Pointer)

Verifikation: gps+0x21C (trip_distance) == TruckPilot-SHM nav_distance_m.
"""
from __future__ import annotations

import mmap
import struct
import sys
from typing import Optional

try:
    import pymem
    import pymem.process
except ImportError:
    print("pymem fehlt: pip install pymem")
    sys.exit(1)

PROC = "eurotrucks2.exe"

# mov rcx, [rip+disp]; lea rdx, [rbp-0x49]; mov rax, [rcx]; call [rax+0x170]
# Treffer: RVA 0x47287E und 0x697910 (beide -> mod+0x33C0548)
AOB_GAME_CTRL_LOAD = bytes(
    [0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01, 0xFF, 0x90, 0x70, 0x01, 0x00, 0x00]
)

# Eindeutiger Code-Anker (1 Treffer); Fallback nur zur Diagnose, nicht zur Auflösung.
AOB_MOVSS_GPS_23C = bytes([0xF3, 0x0F, 0x11, 0x86, 0x3C, 0x02, 0x00, 0x00])

# Bekannte statische Offsets (RVA relativ zu eurotrucks2.exe), Session-unabhängig.
RVA_GAME_CTRL_SLOT = 0x33C0548
GPS_OFFSET_IN_GAME_CTRL = 0x3E30
OFF_GPS_TRIP_DIST = 0x21C

SHM_TAG = r"Local\TruckPilotTelemetry"
SHM_NAV_DIST_OFF = 196  # nav_distance_m (float)


def _mask_match(data: bytes, pattern: bytes) -> bool:
    if len(data) != len(pattern):
        return False
    for a, b in zip(data, pattern):
        if b != 0x00 and a != b:
            return False
    return True


def scan_aob(haystack: bytes, pattern: bytes) -> list[int]:
    n = len(pattern)
    if n == 0 or n > len(haystack):
        return []
    return [i for i in range(len(haystack) - n + 1) if _mask_match(haystack[i : i + n], pattern)]


def rip_resolve(insn_rva: int, insn_len: int, disp_off: int, disp: int) -> int:
    return insn_rva + insn_len + disp


def read_shm_nav_distance() -> Optional[float]:
    try:
        mm = mmap.mmap(-1, 256, tagname=SHM_TAG, access=mmap.ACCESS_READ)
        val = struct.unpack_from("<f", mm.read(256), SHM_NAV_DIST_OFF)[0]
        mm.close()
        return val
    except OSError:
        return None


def resolve_gps_manager(pm: pymem.Pymem, *, verify_shm: bool = True) -> int:
    """
    Liefert die aktuelle gps_manager-Basis (Heap-Adresse, ASLR) ohne manuellen Scan.

    Raises:
        RuntimeError: kein Treffer, ungültiger Pointer oder SHM-Mismatch.
    """
    mod = pymem.process.module_from_name(pm.process_handle, PROC)
    base = mod.lpBaseOfDll
    image = pm.read_bytes(base, mod.SizeOfImage)
    hits = scan_aob(image, AOB_GAME_CTRL_LOAD)

    if not hits:
        raise RuntimeError("AOB_GAME_CTRL_LOAD: kein Treffer im Modul")

    candidates: list[tuple[int, int, int]] = []
    for rva in hits:
        disp = int.from_bytes(image[rva + 3 : rva + 7], "little", signed=True)
        slot_rva = rip_resolve(rva, 7, 3, disp)
        slot_addr = base + slot_rva
        try:
            game_ctrl = pm.read_ulonglong(slot_addr)
        except Exception as exc:
            raise RuntimeError(f"Singleton-Slot 0x{slot_rva:X} nicht lesbar") from exc
        if not (0x10000 < game_ctrl < 0x7FFFFFFFFFFF):
            continue
        gps = game_ctrl + GPS_OFFSET_IN_GAME_CTRL
        candidates.append((gps, game_ctrl, slot_rva))

    if not candidates:
        raise RuntimeError("AOB-Treffer, aber kein gültiger game_ctrl-Pointer")

    # Alle bekannten Treffer zeigen auf denselben Slot; erste gültige Kandidaten reicht.
    gps, game_ctrl, slot_rva = candidates[0]

    if verify_shm:
        nav = read_shm_nav_distance()
        if nav is not None:
            trip = pm.read_float(gps + OFF_GPS_TRIP_DIST)
            if abs(trip - nav) > 2.0:
                raise RuntimeError(
                    f"SHM-Verifikation fehlgeschlagen: gps+0x21C={trip:.1f} "
                    f"SHM={nav:.1f} (gps=0x{gps:X} game_ctrl=0x{game_ctrl:X})"
                )

    return gps


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="gps_manager per AOB auflösen (ETS2 1.59)")
    parser.add_argument("--known", type=lambda x: int(x, 0), help="Bekannte gps-Basis zum Vergleich")
    parser.add_argument("--no-shm", action="store_true", help="SHM-Check überspringen")
    args = parser.parse_args()

    pm = pymem.Pymem(PROC)
    mod = pymem.process.module_from_name(pm.process_handle, PROC)
    base = mod.lpBaseOfDll
    print(f"[+] {PROC} base=0x{base:X}")

    image = pm.read_bytes(base, mod.SizeOfImage)
    aob_hits = scan_aob(image, AOB_GAME_CTRL_LOAD)
    movss_hits = scan_aob(image, AOB_MOVSS_GPS_23C)
    print(f"[+] AOB_GAME_CTRL_LOAD: {len(aob_hits)} Treffer {[hex(h) for h in aob_hits]}")
    print(f"[+] AOB_MOVSS_GPS_23C:  {len(movss_hits)} Treffer {[hex(h) for h in movss_hits]}")

    gps = resolve_gps_manager(pm, verify_shm=not args.no_shm)
    game_ctrl = pm.read_ulonglong(base + RVA_GAME_CTRL_SLOT)
    trip = pm.read_float(gps + OFF_GPS_TRIP_DIST)
    nav = read_shm_nav_distance()

    print(f"[+] game_ctrl slot  mod+0x{RVA_GAME_CTRL_SLOT:X} -> 0x{game_ctrl:X}")
    print(f"[+] gps_manager      = game_ctrl + 0x{GPS_OFFSET_IN_GAME_CTRL:X} = 0x{gps:X}")
    print(f"[+] trip_distance    gps+0x21C = {trip:.1f} m")
    if nav is not None:
        print(f"[+] SHM nav_distance = {nav:.1f} m  match={abs(trip - nav) < 2}")

    if args.known is not None:
        ok = gps == args.known
        print(f"[+] known gps 0x{args.known:X}  {'MATCH' if ok else 'MISMATCH'}")
        return 0 if ok else 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
