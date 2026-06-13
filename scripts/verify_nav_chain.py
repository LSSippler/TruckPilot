#!/usr/bin/env python3
"""Verify gps_manager -> route_task pointer chain (ETS2 1.59)."""
import sys

import ijson
import pymem

GPS = 0x247A2A97C20
SRS = 0x24D5EC10B40
RT = 0x24C5D5B7838
RT_REF = RT - 0x18
ARR = 0x24C5D4A5AD0


def main():
    pm = pymem.Pymem("eurotrucks2.exe")

    def u64(a):
        try:
            return pm.read_ulonglong(a)
        except Exception:
            return None

    p0 = GPS
    p1 = u64(p0 + 0x08)
    p2 = u64(p1 + 0x58) if p1 else None
    p3 = u64(p2 + 0x2C0) if p2 else None
    val = u64(p3 + 0x1A8) if p3 else None

    print("=== Chain resolution ===")
    print(f"gps           = 0x{p0:X}")
    ok1 = p1 == SRS
    print(f"gps+0x08      = 0x{p1:X}  (srs 0x{SRS:X})  {'OK' if ok1 else 'FAIL'}")
    print(f"srs+0x58      = 0x{p2:X}" if p2 else "srs+0x58      FAIL")
    print(f"...+0x2C0     = 0x{p3:X}" if p3 else "...+0x2C0     FAIL")
    print(f"...+0x1A8     = 0x{val:X}" if val else "...+0x1A8     FAIL")
    ok_ref = val == RT_REF
    print(f"expect RT_REF = 0x{RT_REF:X}  {'OK' if ok_ref else 'FAIL'}")

    rt = val + 0x18 if ok_ref else None
    print(f"route_task    = 0x{rt:X}" if rt else "route_task    FAIL")
    arr = u64(rt + 0x50) if rt else None
    print(f"rt+0x50       = 0x{arr:X}" if arr else "rt+0x50       FAIL")
    ok_arr = arr == ARR
    print(f"expect array  = 0x{ARR:X}  {'OK' if ok_arr else 'FAIL'}")

    if arr:
        graph = set()
        with open("graph.json", "rb") as f:
            for i, n in enumerate(ijson.items(f, "nodes.item")):
                graph.add(int(n["uid"]))
                if i > 500_000:
                    break
        hits = sum(
            1 for i in range(37)
            if (uid := u64(arr + i * 0x40 + 0x30)) and uid in graph
        )
        print(f"UID graph hits = {hits}/37")

    ok = ok1 and ok_ref and ok_arr
    print(f"\nCHAIN {'VERIFIED' if ok else 'BROKEN'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
