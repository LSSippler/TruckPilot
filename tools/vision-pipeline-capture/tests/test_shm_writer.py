"""Tests for the SHM writer: header round-trip + sequence-lock semantics."""

from __future__ import annotations

import threading
import time

import pytest

from vision_pipeline_capture import HEADER_BYTES, HEADER_MAGIC, HEADER_VERSION
from vision_pipeline_capture.shm_writer import (
    ShmFrameWriter,
    _HEADER_STRUCT,
    read_frame_with_retry,
    read_header,
)


def test_header_size_is_64_bytes() -> None:
    assert _HEADER_STRUCT.size == HEADER_BYTES == 64


def test_header_roundtrip(tmp_path) -> None:  # noqa: ANN001
    writer = ShmFrameWriter(name="tp-test-roundtrip", buffer_bytes=64 + 4096)
    try:
        jpeg = b"\xff\xd8\xff\xe0" + b"PAYLOAD" + b"\xff\xd9"
        committed = writer.write_frame(width=1920, height=1080, jpeg_bytes=jpeg, timestamp_us=123456789)
        assert committed == 2  # first commit
        # peek directly at the mmap
        magic, version, seq, ts_us, w, h, jpeg_size = read_header(writer._mm)
        assert magic == HEADER_MAGIC
        assert version == HEADER_VERSION
        assert seq == 2
        assert seq % 2 == 0
        assert ts_us == 123456789
        assert (w, h) == (1920, 1080)
        assert jpeg_size == len(jpeg)
        writer._mm.seek(HEADER_BYTES)
        assert writer._mm.read(jpeg_size) == jpeg
    finally:
        writer.close()


def test_sequence_lock_monotonic() -> None:
    writer = ShmFrameWriter(name="tp-test-seq", buffer_bytes=64 + 4096)
    try:
        seqs = [writer.write_frame(8, 8, b"X" * 16) for _ in range(5)]
        # each commit must be even and increase by exactly 2
        assert seqs == [2, 4, 6, 8, 10]
        assert all(s % 2 == 0 for s in seqs)
    finally:
        writer.close()


def test_read_frame_with_retry_returns_committed() -> None:
    writer = ShmFrameWriter(name="tp-test-read", buffer_bytes=64 + 4096)
    try:
        payload = b"\xff\xd8" + b"jpegdata" * 8 + b"\xff\xd9"
        writer.write_frame(640, 480, payload, timestamp_us=42)
        out = read_frame_with_retry(writer._mm)
        assert out is not None
        seq, ts_us, w, h, jpeg = out
        assert seq == 2 and ts_us == 42 and (w, h) == (640, 480)
        assert jpeg == payload
    finally:
        writer.close()


def test_reader_observes_no_torn_frames_under_concurrent_writes() -> None:
    """Background writer hammers the SHM; reader must never see an odd seq committed."""
    writer = ShmFrameWriter(name="tp-test-concurrent", buffer_bytes=64 + 65536)
    stop = threading.Event()
    n_writes = {"v": 0}

    def _writer_loop() -> None:
        payload = b"\xff\xd8" + b"A" * 4096 + b"\xff\xd9"
        while not stop.is_set():
            writer.write_frame(800, 600, payload, timestamp_us=int(time.monotonic() * 1e6))
            n_writes["v"] += 1

    th = threading.Thread(target=_writer_loop, daemon=True)
    th.start()
    try:
        t0 = time.monotonic()
        observed = 0
        invalid = 0
        while time.monotonic() - t0 < 0.5:
            out = read_frame_with_retry(writer._mm, max_attempts=64)
            if out is None:
                continue
            seq, _, _, _, jpeg = out
            observed += 1
            if seq % 2 != 0:
                invalid += 1
            # payload must start/end with JPEG markers
            if not (jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"):
                invalid += 1
    finally:
        stop.set()
        th.join(timeout=1.0)
        writer.close()

    assert observed > 10, f"reader observed too few frames: {observed}"
    assert invalid == 0, f"reader observed {invalid} torn/odd frames out of {observed}"
    assert n_writes["v"] > observed, "writer should outrun the reader"


def test_oversize_payload_rejected() -> None:
    writer = ShmFrameWriter(name="tp-test-oversize", buffer_bytes=64 + 1024)
    try:
        with pytest.raises(ValueError):
            writer.write_frame(1, 1, b"X" * 2048)
    finally:
        writer.close()
