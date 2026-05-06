"""Tests for header parsing."""

from __future__ import annotations

import struct
import io
import pytest
from ets2_hashfs.header import HashFsHeader, MAGIC, CITY_MAGIC, HEADER_SIZE
from ets2_hashfs.exceptions import InvalidMagic, UnsupportedVersion


def build_header(**overrides: int) -> bytes:
    """Build a synthetic header buffer for testing."""
    salt = overrides.get("salt", 0)
    num_entries = overrides.get("num_entries", 1000)
    entry_table_length = overrides.get("entry_table_length", 5000)
    num_metadata = overrides.get("num_metadata", 2000)
    metadata_table_length = overrides.get("metadata_table_length", 8000)
    entry_table_start = overrides.get("entry_table_start", 4096)
    metadata_table_start = overrides.get("metadata_table_start", 50000)
    platform = overrides.get("platform", 0)

    buf = MAGIC
    buf += struct.pack("<H", 2)  # version
    buf += struct.pack("<H", salt)
    buf += CITY_MAGIC
    buf += struct.pack("<I", num_entries)
    buf += struct.pack("<I", entry_table_length)
    buf += struct.pack("<I", num_metadata)
    buf += struct.pack("<I", metadata_table_length)
    buf += struct.pack("<Q", entry_table_start)
    buf += struct.pack("<Q", metadata_table_start)
    buf += struct.pack("<I", 0)  # security_descriptor_offset
    buf += struct.pack("<B", platform)
    return buf


def test_valid_header() -> None:
    """A well-formed header should parse correctly."""
    buf = build_header(salt=42, num_entries=9999)
    f = io.BytesIO(buf)
    h = HashFsHeader.read(f)
    assert h.salt == 42
    assert h.num_entries == 9999


def test_invalid_magic() -> None:
    """Wrong magic must raise InvalidMagic."""
    buf = b"XXXX" + bytes(HEADER_SIZE - 4)
    with pytest.raises(InvalidMagic):
        HashFsHeader.read(io.BytesIO(buf))


def test_wrong_version() -> None:
    """Version != 2 must raise UnsupportedVersion."""
    buf = MAGIC + struct.pack("<H", 1) + bytes(HEADER_SIZE - 6)
    with pytest.raises(UnsupportedVersion):
        HashFsHeader.read(io.BytesIO(buf))


def test_header_size() -> None:
    """Header must be exactly 49 bytes."""
    assert HEADER_SIZE == 49
    assert len(build_header()) == 49
