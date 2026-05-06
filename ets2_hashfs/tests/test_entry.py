"""Tests for MainMetadata bit-unpacking."""

from __future__ import annotations

import struct
import pytest
from ets2_hashfs.entry import MainMetadata, _unpack_28bit, parse_metadata


def test_unpack_28bit_simple() -> None:
    """Known 4-byte input must unpack correctly."""
    # Compressed_size = 0x12345 → bytes: 45 23 01  (little-endian low 3 bytes)
    # Byte 3 = 0x00 (flags=0, high bits of value=0)
    buf = bytes([0x45, 0x23, 0x01, 0x00])  # low 3 = 0x012345, byte3=0x00 → flags_nibble=0, high=0
    value, flags = _unpack_28bit(buf, 0)
    assert value == 0x012345
    assert flags == 0


def test_unpack_28bit_with_flags() -> None:
    """Flags nibble is extracted correctly."""
    # value = 0xABCDE (bits 0-23: DE BC 0A, bit 24-27: low nibble of byte3 = 0)
    # flags = 0b1000 (bit 4 set → is_compressed)
    buf = bytes([0xDE, 0xBC, 0x0A, 0x80])  # byte3=0x80 → flags=8, value=0xABCDE
    value, flags = _unpack_28bit(buf, 0)
    assert value == 0xABCDE
    assert flags == 8


def test_main_metadata_bit_unpack() -> None:
    """Construct a known 16-byte MainMetadata and verify all fields."""
    # compressed_size = 0x12345 (28 bits), flags1 = 8 (is_compressed)
    # size          = 0xABCDE (28 bits), flags2 = 0
    # unknown       = 0
    # offset_block  = 42
    meta_bytes = struct.pack(
        "<III I",
        0x12345 | (8 << 28),   # bytes 0-3: compressed_size in low 28 bits, flags1=8 in high nibble
        0xABCDE,                # bytes 4-7: size in low 28 bits, flags2=0
        0,                      # bytes 8-11: unknown
        42,                     # bytes 12-15: offset_block
    )
    meta = parse_metadata(meta_bytes, 0)
    assert meta.compressed_size == 0x12345
    assert meta.is_compressed is True
    assert meta.size == 0xABCDE
    assert meta.offset_block == 42
    assert meta.offset == 42 * 16


def test_main_metadata_uncompressed() -> None:
    """Uncompressed files have is_compressed=False and compressed_size==size."""
    meta_bytes = struct.pack(
        "<III I",
        0x55555,                # compressed_size (flags1=0 → not compressed)
        0x55555,                # size (same)
        0,
        10,
    )
    meta = parse_metadata(meta_bytes, 0)
    assert meta.is_compressed is False
    assert meta.compressed_size == meta.size
