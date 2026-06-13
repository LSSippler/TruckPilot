#!/usr/bin/env python3
"""Kurzdiagnose: TruckPilot SHM lesen (nav_distance_m, Position)."""
import mmap
import struct
import sys

SHM_NAME = "Local\\TruckPilotTelemetry"
OFF_NAV_DIST = 196
OFF_NAV_TIME = 200
OFF_X, OFF_Z = 16, 32


def main():
    try:
        mm = mmap.mmap(-1, 256, tagname=SHM_NAME, access=mmap.ACCESS_READ)
    except OSError as e:
        print(f"SHM nicht gefunden ({e}) — truckpilot_telemetry.dll geladen?")
        sys.exit(1)
    try:
        data = mm.read(256)
        magic, ver, seq = struct.unpack_from("<III", data, 0)
        x = struct.unpack_from("<d", data, OFF_X)[0]
        z = struct.unpack_from("<d", data, OFF_Z)[0]
        nav_dist = struct.unpack_from("<f", data, OFF_NAV_DIST)[0]
        nav_time = struct.unpack_from("<f", data, OFF_NAV_TIME)[0]
        print(f"magic=0x{magic:08X} ver={ver} seq={seq}")
        print(f"pos x={x:.1f} z={z:.1f}")
        print(f"nav_distance_m={nav_dist:.1f}")
        print(f"nav_time_s={nav_time:.1f}")
        if nav_dist < 0:
            print("WARNUNG: keine aktive Route (nav_distance_m < 0)")
        else:
            print(f"CE-Scan: tp_scan_navdist({int(nav_dist)})")
    finally:
        mm.close()


if __name__ == "__main__":
    main()
