"""Minimap SHM writer — sequence-lock, 64-byte header, f32-point payload.

Header layout (64 bytes, little-endian):
    magic        char[4]   "TPM1"
    version      u32       1
    seq          u64       sequence counter (odd=writing, even=committed)
    timestamp_ms u64       UNIX epoch milliseconds
    point_count  u32       number of (f32, f32) points in payload
    confidence   f32       detection confidence [0.0, 1.0]
    reserved     u8[32]    zero-padded

Payload: point_count × (f32 x, f32 y) — pixel coords within ROI.
         Little-endian, tightly packed, max 256 points.
"""

from __future__ import annotations

import mmap
import struct
import sys
import time

from . import (
    BUFFER_BYTES,
    DEFAULT_SHM_NAME,
    HEADER_BYTES,
    HEADER_MAGIC,
    HEADER_VERSION,
    MAX_POINTS,
    POINT_BYTES,
)

# magic[4s] version[I] seq[Q] ts_ms[Q] point_count[I] confidence[f] reserved[32s]
# 4 + 4 + 8 + 8 + 4 + 4 + 32 = 64
_HEADER_STRUCT = struct.Struct("<4sIQQIf32s")
assert _HEADER_STRUCT.size == HEADER_BYTES, f"header struct mismatch: {_HEADER_STRUCT.size}"


class MinimapShmWriter:
    """Producer: writes detected minimap route-line points to Windows named SHM."""

    def __init__(self, name: str = DEFAULT_SHM_NAME) -> None:
        self.name = name
        self._seq = 0

        if sys.platform == "win32":
            self._mm = mmap.mmap(-1, BUFFER_BYTES, tagname=name)
        else:
            self._mm = mmap.mmap(-1, BUFFER_BYTES)

        # Zero-init + write magic/version so reader sees sane "no frame yet".
        self._write_header(seq=0, ts_ms=0, point_count=0, confidence=0.0)

    def write_points(
        self,
        points: list[tuple[float, float]],
        confidence: float,
    ) -> int:
        """Publish detected route-line points. Returns committed seq value."""
        if len(points) > MAX_POINTS:
            points = points[:MAX_POINTS]

        ts_ms = time.time_ns() // 1_000_000  # UNIX epoch ms
        in_progress = self._seq + 1
        committed = self._seq + 2

        # 1) mark write in-progress (odd seq)
        self._write_header(seq=in_progress, ts_ms=0, point_count=0, confidence=0.0)
        self._mm.flush(0, HEADER_BYTES)

        # 2) write payload
        payload = b"".join(struct.pack("<ff", float(x), float(y)) for x, y in points)
        # pad to MAX_POINTS * POINT_BYTES
        payload = payload.ljust(MAX_POINTS * POINT_BYTES, b"\x00")
        self._mm.seek(HEADER_BYTES)
        self._mm.write(payload)

        # 3) commit header (even seq)
        self._write_header(
            seq=committed,
            ts_ms=ts_ms,
            point_count=len(points),
            confidence=float(confidence),
        )
        self._mm.flush(0, HEADER_BYTES)

        self._seq = committed
        return committed

    def close(self) -> None:
        try:
            self._mm.close()
        except Exception:
            pass

    def __enter__(self) -> "MinimapShmWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _write_header(self, seq: int, ts_ms: int, point_count: int, confidence: float) -> None:
        header = _HEADER_STRUCT.pack(
            HEADER_MAGIC,
            HEADER_VERSION,
            seq,
            ts_ms,
            point_count,
            confidence,
            b"\x00" * 32,
        )
        self._mm.seek(0)
        self._mm.write(header)


# ── reader helper (used by tests + Rust compat check) ────────────────────────

def read_header(mm: mmap.mmap) -> tuple[bytes, int, int, int, int, float]:
    """Returns (magic, version, seq, ts_ms, point_count, confidence)."""
    mm.seek(0)
    raw = mm.read(HEADER_BYTES)
    magic, version, seq, ts_ms, point_count, confidence, _reserved = _HEADER_STRUCT.unpack(raw)
    return magic, version, seq, ts_ms, point_count, confidence


def read_points_with_retry(
    mm: mmap.mmap, max_attempts: int = 32
) -> tuple[int, int, float, list[tuple[float, float]]] | None:
    """Reader-side helper. Returns (seq, ts_ms, confidence, points) or None."""
    for _ in range(max_attempts):
        magic, version, seq_before, ts_ms, point_count, confidence = read_header(mm)
        if magic != HEADER_MAGIC or version != HEADER_VERSION:
            return None
        if seq_before == 0 or seq_before & 1:
            continue
        if point_count == 0:
            return seq_before, ts_ms, confidence, []
        if point_count > MAX_POINTS:
            return None  # corrupt

        mm.seek(HEADER_BYTES)
        payload = mm.read(point_count * POINT_BYTES)
        _, _, seq_after, *_ = read_header(mm)
        if seq_after != seq_before:
            continue
        points = [
            struct.unpack_from("<ff", payload, i * POINT_BYTES)
            for i in range(point_count)
        ]
        return seq_before, ts_ms, confidence, list(points)
    return None
