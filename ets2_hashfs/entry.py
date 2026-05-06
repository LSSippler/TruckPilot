"""HashFS v2 entry structures.

Entry table (16 bytes each, zlib-compressed)::

    Offset  Size  Field
    ──────  ────  ──────────────────
    0x00    8     hash           u64 LE (CityHash64 of path)
    0x08    4     metadata_index u32 LE (index into metadata, ×4 for byte offset)
    0x0C    2     metadata_count u16 LE (number of metadata chunks)
    0x0E    2     flags          u16 LE (bit0 = IsDirectory)

MainMetadata (16 bytes total, addressed via metadata_index * 4)::

    ┌──────────────────────────────────────────────────────────┐
    │ Bytes 0-2 + byte3[3:0]  → compressed_size   (28 bits)   │
    │ Byte 3[7:4]             → flags1            (4 bits)    │
    │   bit 4                 → is_compressed                  │
    │ Bytes 4-6 + byte7[3:0]  → size              (28 bits)   │
    │ Byte 7[7:4]             → flags2            (4 bits)    │
    │ Bytes 8-11              → unknown           (u32)       │
    │ Bytes 12-15             → offset_block      (u32)       │
    │                            real_byte_offset = off * 16  │
    └──────────────────────────────────────────────────────────┘

Metadata is addressed in 4-byte blocks. Each entry starts with
`metadata_count` chunk headers (4 bytes each), followed by the
MainMetadata for chunk type 128 (Plain) or 129 (Directory).
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional


class ChunkType(IntEnum):
    """Known metadata chunk type identifiers."""
    PLAIN = 128
    DIRECTORY = 129
    IMAGE = 1       # packed .tobj/.dds — out of scope
    SAMPLE = 2
    MIP_TAIL = 132


@dataclass
class EntryTableEntry:
    """A single entry in the decompressed entry table."""

    hash: int
    metadata_index: int
    metadata_count: int
    flags: int

    @property
    def is_directory(self) -> bool:
        return (self.flags & 1) != 0


@dataclass
class MainMetadata:
    """Metadata for a single file entry (Plain or Directory chunk)."""

    compressed_size: int   # 28 bits — number of compressed bytes on disk
    size: int              # 28 bits — uncompressed size
    is_compressed: bool    # from flags1[4]
    offset_block: int      # u32 — byte offset = offset_block * 16

    @property
    def offset(self) -> int:
        """Absolute byte offset to the file data."""
        return self.offset_block * 16


def _unpack_28bit(buf: bytes, offset: int) -> tuple[int, int]:
    """Unpack a 28-bit value and return (value, flags_nibble).

    Bytes at offset..offset+2 are the low 24 bits.
    Byte at offset+3 provides bits 24-27 in its low nibble,
    and a 4-bit flags field in its high nibble.
    """
    low = buf[offset] | (buf[offset + 1] << 8) | (buf[offset + 2] << 16)
    high_and_flags = buf[offset + 3]
    value = low | ((high_and_flags & 0x0F) << 24)
    flags = (high_and_flags & 0xF0) >> 4
    return value, flags


def parse_metadata(meta_bytes: bytes, index: int) -> MainMetadata:
    """Parse MainMetadata from the metadata table at the given block index."""
    off = index * 4
    if off + 16 > len(meta_bytes):
        raise ValueError(f"metadata index {index} out of range (off={off}, len={len(meta_bytes)})")

    compressed_size, flags1 = _unpack_28bit(meta_bytes, off)
    is_compressed = bool(flags1 & 0x08)  # bit 4 → 0b1000 = 8

    size, _flags2 = _unpack_28bit(meta_bytes, off + 4)
    # unknown = struct.unpack_from("<I", meta_bytes, off + 8)[0]
    offset_block = struct.unpack_from("<I", meta_bytes, off + 12)[0]

    return MainMetadata(
        compressed_size=compressed_size,
        size=size,
        is_compressed=is_compressed,
        offset_block=offset_block,
    )


def parse_entry_table(raw: bytes) -> List[EntryTableEntry]:
    """Parse the decompressed entry table into a list of entries.

    Each entry is exactly 16 bytes.
    """
    entries: list[EntryTableEntry] = []
    for i in range(0, len(raw), 16):
        chunk = raw[i:i + 16]
        if len(chunk) < 16:
            break
        hash_val = struct.unpack_from("<Q", chunk, 0)[0]
        meta_idx = struct.unpack_from("<I", chunk, 8)[0]
        meta_cnt = struct.unpack_from("<H", chunk, 12)[0]
        flags = struct.unpack_from("<H", chunk, 14)[0]
        entries.append(EntryTableEntry(
            hash=hash_val,
            metadata_index=meta_idx,
            metadata_count=meta_cnt,
            flags=flags,
        ))
    return entries


def read_chunk_types(meta_bytes: bytes, index: int, count: int) -> List[int]:
    """Read chunk type headers for a metadata entry."""
    types: list[int] = []
    off = index * 4
    for _ in range(count):
        if off + 4 > len(meta_bytes):
            break
        chunk_type = meta_bytes[off + 3]  # byte 3 is the type identifier
        types.append(chunk_type)
        off += 4
    return types
