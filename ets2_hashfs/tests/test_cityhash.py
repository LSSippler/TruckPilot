"""Tests for CityHash64 implementation."""

from __future__ import annotations

import pytest
from ets2_hashfs.cityhash import cityhash64


def test_empty_string() -> None:
    """cityhash64(b"") must equal the known Google reference value."""
    assert cityhash64(b"") == 0x9AE16A3B2F90404F


def test_deterministic() -> None:
    """Same input must produce the same hash."""
    for s in [b"", b"a", b"abc", b"hello world", b"x" * 100]:
        assert cityhash64(s) == cityhash64(s)


def test_different_inputs() -> None:
    """Different inputs should produce different hashes."""
    hashes = {cityhash64(s) for s in [b"", b"a", b"abc", b"hello", b"world"]}
    assert len(hashes) == 5


def test_non_zero() -> None:
    """Any non-empty string should produce a non-zero hash."""
    assert cityhash64(b"anything") != 0


def test_long_string() -> None:
    """Strings longer than 64 bytes must also work (chunk loop)."""
    long_data = b"x" * 200
    h = cityhash64(long_data)
    assert h != 0
    # Must be deterministic
    assert cityhash64(long_data) == h


def test_all_length_ranges() -> None:
    """Test each branch: 0-16, 17-32, 33-64, >64 bytes."""
    for L in [0, 1, 3, 4, 8, 15, 16, 17, 31, 32, 33, 63, 64, 65, 100, 200]:
        data = bytes(range(L % 256)) * ((L // 256) + 1)
        data = data[:L]
        h = cityhash64(data)
        assert isinstance(h, int)
        assert 0 <= h <= 0xFFFFFFFFFFFFFFFF
