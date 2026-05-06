"""HashFS v2 header parsing.

Header layout (45 bytes total after magic+version at offset 6)::

    Offset  Size  Field
    ──────  ────  ─────────────────────────────────
    0x00    4     magic          b"SCS#" (0x53,0x43,0x53,0x23)
    0x04    2     version        u16 LE (must be 2)
    0x06    2     salt           u16 LE
    0x08    4     hash_method    b"CITY" (0x43,0x49,0x54,0x59)
    0x0C    4     num_entries    u32 LE  (entry table entries)
    0x10    4     entry_table_length   u32 LE  (compressed bytes)
    0x14    4     num_metadata   u32 LE  (metadata blocks)
    0x18    4     metadata_table_length u32 LE  (compressed bytes)
    0x1C    8     entry_table_start     u64 LE  (absolute file offset)
    0x24    8     metadata_table_start  u64 LE  (absolute file offset)
    0x2C    4     security_descriptor_offset  u32 LE
    0x30    1     platform        u8
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import BinaryIO

from .exceptions import InvalidMagic, UnsupportedVersion

MAGIC = b"SCS#"
CITY_MAGIC = b"CITY"
HEADER_SIZE = 0x31  # 49 bytes


@dataclass
class HashFsHeader:
    """Parsed HashFS v2 header."""

    salt: int
    num_entries: int
    entry_table_length: int
    num_metadata: int
    metadata_table_length: int
    entry_table_start: int
    metadata_table_start: int
    platform: int

    @classmethod
    def read(cls, f: BinaryIO) -> "HashFsHeader":
        """Read and validate the HashFS v2 header from an open file."""
        buf = f.read(HEADER_SIZE)
        if len(buf) < HEADER_SIZE:
            raise InvalidMagic(b"too short")

        if buf[0:4] != MAGIC:
            raise InvalidMagic(buf[0:4])

        version = struct.unpack_from("<H", buf, 4)[0]
        if version != 2:
            raise UnsupportedVersion(version)

        salt = struct.unpack_from("<H", buf, 6)[0]
        hm = buf[8:12]
        if hm != CITY_MAGIC:
            raise InvalidMagic(hm)

        num_entries = struct.unpack_from("<I", buf, 12)[0]
        entry_table_length = struct.unpack_from("<I", buf, 16)[0]
        num_metadata = struct.unpack_from("<I", buf, 20)[0]
        metadata_table_length = struct.unpack_from("<I", buf, 24)[0]
        entry_table_start = struct.unpack_from("<Q", buf, 28)[0]
        metadata_table_start = struct.unpack_from("<Q", buf, 36)[0]
        # security_descriptor_offset = struct.unpack_from("<I", buf, 44)[0]
        platform = buf[48]

        return cls(
            salt=salt,
            num_entries=num_entries,
            entry_table_length=entry_table_length,
            num_metadata=num_metadata,
            metadata_table_length=metadata_table_length,
            entry_table_start=entry_table_start,
            metadata_table_start=metadata_table_start,
            platform=platform,
        )
