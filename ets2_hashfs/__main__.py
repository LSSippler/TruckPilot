"""CLI utilities for ets2_hashfs.

Usage:
    python -m ets2_hashfs <path/to/base.scs>
    python -m ets2_hashfs extract <path/to/base.scs> --sectors --out <dir>
"""

from __future__ import annotations

import os
import sys
from .reader import HashFsReader


def print_usage() -> None:
    print("Usage: python -m ets2_hashfs <path/to/base.scs> [--debug-hash <path>]")
    print("       python -m ets2_hashfs extract <path/to/base.scs> --sectors --out <dir>")


def main() -> None:
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)

    if sys.argv[1] in {"-h", "--help", "help"}:
        print_usage()
        sys.exit(0)

    if sys.argv[1] == "extract":
        run_extract(sys.argv[2:])
        return

    path = sys.argv[1]
    debug_path = None
    if len(sys.argv) >= 4 and sys.argv[2] == "--debug-hash":
        debug_path = sys.argv[3]

    print(f"Opening: {path}")

    with HashFsReader(path) as archive:
        h = archive.header
        print(f"  Entries (total):      {archive.num_entries}")
        print(f"  Entry table length:   {h.entry_table_length} bytes (compressed)")
        print(f"  Metadata blocks:      {h.num_metadata}")
        print(f"  Metadata length:      {h.metadata_table_length} bytes (compressed)")

        if debug_path:
            info = archive.debug_hash(debug_path)
            print(f"\n  Debug hash for: {debug_path}")
            print(f"    Normalized:     {info['normalized_repr']}")
            print(f"    Salt:           {info['salt']}")
            print(f"    Computed hash:  {info['hash']}")
            print(f"    Found in table: {info['found']}")
            if not info['found']:
                print(f"    (Entry hashes start with: 0x{archive._entries[0].hash:016x})")

        entries = archive.list_entries()
        if entries:
            sizes = [e[1] for e in entries if e[1] > 0]
            compressed = [e[1] for e in entries if e[2]]
            plain = [e[1] for e in entries if not e[2]]
            if sizes:
                sizes.sort()
                print(f"  Plain files:          {len(plain)}")
                print(f"  Compressed files:     {len(compressed)}")
                print(f"  Size min/median/max:  {sizes[0]} / {sizes[len(sizes)//2]} / {sizes[-1]}")

        for known in ["def/world/road.sii", "def/world/prefab.sii"]:
            if archive.has_file(known):
                data = archive.read_file(known)
                print(f"  {known}: {len(data)} bytes — starts: {data[:60]}")
            else:
                print(f"  {known}: NOT FOUND")

        found_sectors = 0
        for x in range(-5, 6):
            for z in range(-5, 6):
                path = f"map/europe/sec{'+' if x >= 0 else ''}{x:04}{'+' if z >= 0 else ''}{z:04}.base"
                if archive.has_file(path):
                    data = archive.read_file(path)
                    found_sectors += 1
                    if found_sectors <= 3:
                        print(f"  {path}: {len(data)} bytes")
        print(f"  Map sectors found:    {found_sectors}")


def run_extract(args: list[str]) -> None:
    if not args:
        print("Usage: python -m ets2_hashfs extract <path/to/base.scs> --sectors --out <dir>")
        sys.exit(1)

    base_path = args[0]
    out_dir = None
    sectors_only = False

    i = 1
    while i < len(args):
        if args[i] == "--sectors":
            sectors_only = True
            i += 1
        elif args[i] == "--out" and i + 1 < len(args):
            out_dir = args[i + 1]
            i += 2
        else:
            i += 1

    if not out_dir:
        out_dir = os.path.join(os.getcwd(), "ets2_sectors")

    os.makedirs(out_dir, exist_ok=True)

    with HashFsReader(base_path) as archive:
        if sectors_only:
            extract_sectors(archive, out_dir)
        else:
            print("Nothing to extract: pass --sectors")


def extract_sectors(archive: HashFsReader, out_dir: str) -> None:
    entries = []
    try:
        entries = archive.read_directory("map/europe")
    except Exception as exc:  # pragma: no cover - best effort
        print(f"Failed to read map/europe directory: {exc}")
        entries = []

    count = 0
    if not entries:
        try:
            raw = archive.read_file("map/europe")
            entries = parse_dir_listing(raw)
            if not entries:
                import zlib
                entries = parse_dir_listing(zlib.decompress(raw))
        except Exception:
            entries = []

    names = [name for name in entries if name.endswith(".base")]

    if not names:
        names = probe_sector_grid(archive, 200)

    for name in names:
        path = f"map/europe/{name}"
        data = archive.read_file(path)
        out_path = os.path.join(out_dir, name)
        with open(out_path, "wb") as f:
            f.write(data)
        count += 1

    print(f"Extracted {count} sector files to {out_dir}")


def probe_sector_grid(archive: HashFsReader, radius: int) -> list[str]:
    names: list[str] = []
    for x in range(-radius, radius + 1):
        for z in range(-radius, radius + 1):
            base_name = f"sec{'+' if x >= 0 else ''}{x:04}{'+' if z >= 0 else ''}{z:04}"
            name = f"{base_name}.base"
            if archive.has_file(f"map/europe/{name}"):
                names.append(name)
    return names


def parse_dir_listing(data: bytes) -> list[str]:
    if len(data) < 4:
        return []
    count = int.from_bytes(data[0:4], "little")
    if count <= 0 or count > 200000:
        return []
    if 4 + count > len(data):
        return []
    lengths = data[4:4 + count]
    off = 4 + count
    result: list[str] = []
    for ln in lengths:
        if off + ln > len(data):
            break
        s = data[off:off + ln].decode("utf-8", errors="replace")
        if s.startswith("/"):
            s = s[1:]
        result.append(s)
        off += ln
    return result


if __name__ == "__main__":
    main()
