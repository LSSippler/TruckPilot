#!/usr/bin/env python3
"""
TruckPilot SHM reader — polls Local\\TruckPilotControls every 100 ms.

Shows the raw values the DLL sees each frame:
  magic  version  seq  active  steering  throttle  brake  clutch

Run while ETS2 + TruckPilot DLL are loaded (after scs_input_init fires).
Press Ctrl-C to stop.

Layout (32 bytes, little-endian, all naturally aligned):
  offset  0: u32 magic    (0x54504354 = "TPCT")
  offset  4: u32 version  (1)
  offset  8: u32 sequence (monotonic, incremented by daemon on each write)
  offset 12: u32 active   (1 = autopilot on, 0 = passthrough)
  offset 16: f32 steering (-1.0 .. +1.0)
  offset 20: f32 throttle (0.0 .. 1.0)
  offset 24: f32 brake    (0.0 .. 1.0)
  offset 28: f32 clutch   (0.0 .. 1.0)
"""
import ctypes
import struct
import sys
import time

SHM_NAME    = "Local\\TruckPilotControls"
SHM_SIZE    = 32
SHM_MAGIC   = 0x5450_4354  # "TPCT"
SHM_VERSION = 1
FILE_MAP_READ = 0x0004

kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]


def open_shm() -> tuple[int, int]:
    handle = kernel32.OpenFileMappingW(FILE_MAP_READ, False, SHM_NAME)
    if not handle:
        err = kernel32.GetLastError()
        print(f"OpenFileMappingW failed (err={err}) — is ETS2 running with the TruckPilot DLL loaded?")
        sys.exit(1)
    ptr = kernel32.MapViewOfFile(handle, FILE_MAP_READ, 0, 0, SHM_SIZE)
    if not ptr:
        err = kernel32.GetLastError()
        print(f"MapViewOfFile failed (err={err})")
        kernel32.CloseHandle(handle)
        sys.exit(1)
    return handle, ptr


def main() -> None:
    handle, ptr = open_shm()
    buf = (ctypes.c_char * SHM_SIZE).from_address(ptr)

    hdr = f"{'tick':>6}  {'magic':>12}  {'ver':>3}  {'seq':>10}  {'active':>6}  " \
          f"{'steering':>9}  {'throttle':>9}  {'brake':>9}  {'clutch':>9}"
    print(hdr)
    print("-" * len(hdr))

    prev_seq = -1
    tick = 0
    try:
        while True:
            raw = bytes(buf)
            magic, version, seq, active, steering, throttle, brake, clutch = \
                struct.unpack_from("<IIIIffff", raw)

            magic_str = "OK" if magic == SHM_MAGIC else f"BAD={magic:#010x}"
            seq_flag  = " *" if seq != prev_seq else "  "
            print(
                f"{tick:>6}  {magic_str:>12}  {version:>3}  {seq:>10}{seq_flag}  "
                f"{active:>6}  {steering:>+9.4f}  {throttle:>9.4f}  "
                f"{brake:>9.4f}  {clutch:>9.4f}"
            )
            prev_seq = seq
            tick += 1
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        kernel32.UnmapViewOfFile(ptr)
        kernel32.CloseHandle(handle)


if __name__ == "__main__":
    main()
