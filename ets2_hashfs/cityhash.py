"""CityHash64 — Google's 2011 hash function as used by SCS HashFS v2.

Exact Python port of the C# implementation from TruckLib (CityHash.cs),
which is itself a port of Google's original cityhash-c.

Test vectors (from TruckLib unit tests):
    cityhash64("käsefondue.txt" UTF-8) = 0x77F93071B1668154
    cityhash64("*")                    = 0x0DAC6B40444905D0
    cityhash64("*def/world/model.tests.sii") = 0x3C6369BC6EFDD668
    cityhash64("")                     = 0x9AE16A3B2F90404F
"""

from __future__ import annotations

import struct

K0: int = 0xC3A5C85C97CB3127
K1: int = 0xB492B66FBE98F273
K2: int = 0x9AE16A3B2F90404F
K3: int = 0xC949D7C7509E6557  # ← different from common K3=0x9DDFEA08EB382D69
MASK64: int = 0xFFFFFFFFFFFFFFFF


def _u32le(data: bytes, off: int = 0) -> int:
    return struct.unpack_from("<I", data, off)[0]


def _u64le(data: bytes, off: int = 0) -> int:
    return struct.unpack_from("<Q", data, off)[0]


def _rot64(val: int, shift: int) -> int:
    if shift == 0:
        return val
    return ((val >> shift) | (val << (64 - shift))) & MASK64


def _rot64_at_least_1(val: int, shift: int) -> int:
    """Rotate right — used in HashLen0To16 for len>8."""
    return ((val >> shift) | (val << (64 - shift))) & MASK64


def _shift_mix(val: int) -> int:
    return (val ^ (val >> 47)) & MASK64


def _hash128to64(lo: int, hi: int) -> int:
    """Hash 128 input bits down to 64 bits. Murmur-inspired."""
    k_mul: int = 0x9DDFEA08EB382D69
    a = ((lo ^ hi) * k_mul) & MASK64
    a ^= a >> 47
    b = ((hi ^ a) * k_mul) & MASK64
    b ^= b >> 47
    return (b * k_mul) & MASK64


def _hash_len16(u: int, v: int) -> int:
    return _hash128to64(u, v)


def _hash_len_0_to_16(data: bytes) -> int:
    L = len(data)
    if L > 8:
        a = _u64le(data, 0)
        b = _u64le(data, L - 8)
        return _hash_len16(a, _rot64_at_least_1(b + L, L)) ^ b
    if L >= 4:
        a = _u32le(data, 0)
        return _hash_len16(L + (a << 3), _u32le(data, L - 4))
    if L > 0:
        a = data[0]
        b = data[L >> 1]
        c = data[L - 1]
        y = a + (b << 8)
        z = L + (c << 2)
        return _shift_mix((y * K2) & MASK64 ^ (z * K3) & MASK64) * K2 & MASK64
    return K2


def _hash_len_17_to_32(data: bytes) -> int:
    L = len(data)
    a = (_u64le(data, 0) * K1) & MASK64
    b = _u64le(data, 8)
    c = (_u64le(data, L - 8) * K2) & MASK64
    d = (_u64le(data, L - 16) * K0) & MASK64
    return _hash_len16(
        (_rot64(a - b, 43) + _rot64(c, 30) + d) & MASK64,
        (a + _rot64(b ^ K3, 20) - c + L) & MASK64,
    )


def _hash_len_33_to_64(data: bytes) -> int:
    L = len(data)
    z = _u64le(data, 24)
    a = (_u64le(data, 0) + (L + _u64le(data, L - 16)) * K0) & MASK64
    b = _rot64(a + z, 52)
    c = _rot64(a, 37)
    a = (a + _u64le(data, 8)) & MASK64
    c = (c + _rot64(a, 7)) & MASK64
    a = (a + _u64le(data, 16)) & MASK64
    vf = a + z
    vs = b + _rot64(a, 31) + c
    a = _u64le(data, 16) + _u64le(data, L - 32)
    z = _u64le(data, L - 8)
    b = _rot64(a + z, 52)
    c = _rot64(a, 37)
    a = (a + _u64le(data, L - 24)) & MASK64
    c = (c + _rot64(a, 7)) & MASK64
    a = (a + _u64le(data, L - 16)) & MASK64
    wf = a + z
    ws = b + _rot64(a, 31) + c
    r = _shift_mix(((vf + ws) * K2 + (wf + vs) * K0) & MASK64)
    return _shift_mix((r * K0 + vs) & MASK64) * K2 & MASK64


def _weak_hash_len_32_with_seeds(w: int, x: int, y: int, z: int, a: int, b: int) -> tuple[int, int]:
    a = (a + w) & MASK64
    b = _rot64(b + a + z, 21)
    c = a
    a = (a + x) & MASK64
    a = (a + y) & MASK64
    b = (b + _rot64(a, 44)) & MASK64
    return (a + z) & MASK64, (b + c) & MASK64


def _weak_hash_len_32_with_seeds_buf(data: bytes, off: int, a: int, b: int) -> tuple[int, int]:
    return _weak_hash_len_32_with_seeds(
        _u64le(data, off),
        _u64le(data, off + 8),
        _u64le(data, off + 16),
        _u64le(data, off + 24),
        a, b,
    )


def cityhash64(data: bytes) -> int:
    """Compute the CityHash64 of a byte string.

    Exact port of the C# implementation from TruckLib (CityHash.cs),
    which is Google's original CityHash64 (2011) used by SCS HashFS v2.
    """
    L = len(data)
    if L <= 16:
        return _hash_len_0_to_16(data)
    if L <= 32:
        return _hash_len_17_to_32(data)
    if L <= 64:
        return _hash_len_33_to_64(data)

    # For strings over 64 bytes: hash the end first, then process 64-byte chunks.
    x = _u64le(data, L - 40)
    y = (_u64le(data, L - 16) + _u64le(data, L - 56)) & MASK64
    z = _hash_len16(_u64le(data, L - 48) + L, _u64le(data, L - 24))
    v1, v2 = _weak_hash_len_32_with_seeds_buf(data, L - 64, L, z)
    w1, w2 = _weak_hash_len_32_with_seeds_buf(data, L - 32, (y + K1) & MASK64, x)
    x = (x * K1 + _u64le(data, 0)) & MASK64

    # Decrease len to nearest multiple of 64.
    remaining = (L - 1) & ~63
    pos = 0
    while remaining > 0:
        x = (_rot64((x + y + v1 + _u64le(data, pos + 8)) & MASK64, 37) * K1) & MASK64
        y = (_rot64((y + v2 + _u64le(data, pos + 48)) & MASK64, 42) * K1) & MASK64
        x ^= w2
        y = (y + v1 + _u64le(data, pos + 40)) & MASK64
        z = (_rot64(z + w1, 33) * K1) & MASK64
        v1, v2 = _weak_hash_len_32_with_seeds_buf(data, pos, (v2 * K1) & MASK64, (x + w1) & MASK64)
        w1, w2 = _weak_hash_len_32_with_seeds_buf(data, pos + 32, (z + w2) & MASK64, (y + _u64le(data, pos + 16)) & MASK64)
        z, x = x, z  # swap
        pos += 64
        remaining -= 64

    return _hash_len16(
        _hash_len16(v1, w1) + (_shift_mix(y) * K1 + z) & MASK64,
        _hash_len16(v2, w2) + x,
    )
