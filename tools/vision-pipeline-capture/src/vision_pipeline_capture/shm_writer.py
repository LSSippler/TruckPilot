"""Shared-memory frame producer with sequence-lock semantics.

Header layout (64 bytes, little-endian, packed):
    magic         char[4]   (4)   "TPF1"
    version       u32       (4)   1
    frame_id      u64       (8)   sequence counter; odd while writing, even when committed
    timestamp_us  u64       (8)   monotonic micros, set at commit
    width         u32       (4)
    height        u32       (4)
    jpeg_size     u32       (4)
    reserved      bytes[28] (28)

Payload immediately follows the header: jpeg_size bytes of JPEG data.

Sequence-lock pattern (single writer / many readers):
    1. read seq (currently even -> N)
    2. write N+1 to seq           # odd: in-progress
    3. write width/height/size/timestamp + payload bytes
    4. write N+2 to seq           # even: committed

A reader retries while seq is odd or while seq changes between reading header
and reading payload.
"""

from __future__ import annotations

import logging
import mmap
import struct
import sys
import time
from typing import Final

from . import (
    DEFAULT_BUFFER_BYTES,
    DEFAULT_SHM_NAME,
    HEADER_BYTES,
    HEADER_MAGIC,
    HEADER_VERSION,
)

log = logging.getLogger(__name__)

# struct: <4s I Q Q I I I 28s   ->  4+4+8+8+4+4+4+28 = 64 bytes
_HEADER_STRUCT: Final = struct.Struct("<4sIQQIII28s")
assert _HEADER_STRUCT.size == HEADER_BYTES, f"header struct is {_HEADER_STRUCT.size} bytes, expected {HEADER_BYTES}"


def _resolve_tag(name: str) -> str:
    """On Windows, mmap.tagname maps to Local\\<name>. Other platforms ignore the tag."""
    return name


class ShmFrameWriter:
    """Producer side of the TruckPilot frame SHM buffer.

    The constructor sizes and initializes the mmap region. On Windows, the
    `tagname` parameter routes to a named Local\\ shared section that other
    processes can open by name (the Rust reader uses OpenFileMappingW).
    """

    def __init__(
        self,
        name: str = DEFAULT_SHM_NAME,
        buffer_bytes: int = DEFAULT_BUFFER_BYTES,
    ) -> None:
        if buffer_bytes <= HEADER_BYTES:
            raise ValueError("buffer_bytes must exceed header size")
        self.name = name
        self.buffer_bytes = buffer_bytes
        self._max_payload = buffer_bytes - HEADER_BYTES
        self._seq = 0  # monotonic sequence (even = committed, odd = in-progress)

        if sys.platform == "win32":
            self._mm = mmap.mmap(-1, buffer_bytes, tagname=_resolve_tag(name))
        else:  # posix: anonymous mmap; tests rely on this path
            self._mm = mmap.mmap(-1, buffer_bytes)

        # zero-init header so readers see a sane "no frame yet" state.
        self._mm.seek(0)
        self._mm.write(b"\x00" * HEADER_BYTES)
        # write magic + version once; frame_id stays 0 until first publish.
        self._write_header(seq=0, ts_us=0, w=0, h=0, jpeg_size=0)
        log.info("shm-writer ready: name=%s size=%d bytes", name, buffer_bytes)

    # ----------------------------------------------------------------- API

    @property
    def sequence(self) -> int:
        """Monotonic committed sequence value (even). Frame id = sequence / 2."""
        return self._seq

    def write_frame(
        self,
        width: int,
        height: int,
        jpeg_bytes: bytes,
        timestamp_us: int | None = None,
    ) -> int:
        """Publish a JPEG frame. Returns the committed sequence value."""
        if len(jpeg_bytes) > self._max_payload:
            raise ValueError(
                f"jpeg_bytes ({len(jpeg_bytes)} B) exceeds payload budget ({self._max_payload} B)"
            )
        ts = timestamp_us if timestamp_us is not None else int(time.monotonic() * 1_000_000)

        in_progress = self._seq + 1   # odd
        committed = self._seq + 2     # even

        # 1) mark in-progress
        self._write_header(seq=in_progress, ts_us=0, w=0, h=0, jpeg_size=0)
        self._mm.flush(0, HEADER_BYTES)  # ensure ordering on weak platforms

        # 2) write payload
        self._mm.seek(HEADER_BYTES)
        self._mm.write(jpeg_bytes)

        # 3) write final header + commit seq
        self._write_header(seq=committed, ts_us=ts, w=width, h=height, jpeg_size=len(jpeg_bytes))
        self._mm.flush(0, HEADER_BYTES)

        self._seq = committed
        return committed

    def close(self) -> None:
        try:
            self._mm.close()
        except Exception:  # noqa: BLE001
            pass

    def __enter__(self) -> "ShmFrameWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ----------------------------------------------------------------- helpers

    def _write_header(self, seq: int, ts_us: int, w: int, h: int, jpeg_size: int) -> None:
        header = _HEADER_STRUCT.pack(
            HEADER_MAGIC,
            HEADER_VERSION,
            seq,
            ts_us,
            w,
            h,
            jpeg_size,
            b"\x00" * 28,
        )
        self._mm.seek(0)
        self._mm.write(header)


# --------------------------------------------------------------------- reader (used by tests)


def read_header(mm: mmap.mmap) -> tuple[bytes, int, int, int, int, int, int]:
    """Decode the header. Returns (magic, version, seq, ts_us, w, h, jpeg_size)."""
    mm.seek(0)
    raw = mm.read(HEADER_BYTES)
    magic, version, seq, ts_us, w, h, jpeg_size, _reserved = _HEADER_STRUCT.unpack(raw)
    return magic, version, seq, ts_us, w, h, jpeg_size


def read_frame_with_retry(mm: mmap.mmap, max_attempts: int = 32) -> tuple[int, int, int, int, bytes] | None:
    """Reader-side helper. Returns (seq, ts_us, w, h, jpeg_bytes) or None on giveup."""
    for _ in range(max_attempts):
        magic, version, seq_before, ts_us, w, h, jpeg_size = read_header(mm)
        if magic != HEADER_MAGIC or version != HEADER_VERSION:
            return None
        if seq_before == 0 or seq_before & 1:
            continue  # no frame yet or write in progress
        mm.seek(HEADER_BYTES)
        payload = mm.read(jpeg_size)
        _, _, seq_after, *_ = read_header(mm)
        if seq_after == seq_before:
            return seq_before, ts_us, w, h, payload
        # writer overlapped; retry
    return None
