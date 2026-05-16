"""Video-file -> SHM replay (Phase 6.5h verification).

Reads frames from one or more MP4/MKV files with OpenCV and publishes them
through the same SHM channel the live capture uses ("TruckPilotFrame"). Quick-
hack so the sign-vision plugin can be re-run against the exact pixels of a
recorded ETS2 session, including back-to-back playlists.
"""

from __future__ import annotations

import logging
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2

from . import DEFAULT_JPEG_QUALITY, DEFAULT_SHM_NAME
from .shm_writer import ShmFrameWriter

log = logging.getLogger(__name__)

TARGET_W = 1920
TARGET_H = 1080


@dataclass
class ReplaySummary:
    videos_played: int = 0
    frames_published: int = 0
    duration_s: float = 0.0


def run_replay(
    videos: Iterable[Path],
    fps: float,
    loop: bool,
    shm_name: str = DEFAULT_SHM_NAME,
    jpeg_quality: int = DEFAULT_JPEG_QUALITY,
) -> ReplaySummary:
    """Replay a playlist of videos into SHM. Returns aggregate stats."""
    playlist = list(videos)
    if not playlist:
        raise ValueError("playlist is empty")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    period = 1.0 / max(fps, 0.1)

    stopped = False

    def _stop(*_: object) -> None:
        nonlocal stopped
        stopped = True
        log.info("replay: shutdown requested")

    signal.signal(signal.SIGINT, _stop)

    summary = ReplaySummary()
    next_tick = time.monotonic()

    log.info("replay: playlist of %d video(s), loop=%s, fps=%.1f", len(playlist), loop, fps)

    try:
        with ShmFrameWriter(name=shm_name) as writer:
            pass_idx = 0
            while not stopped:
                pass_idx += 1
                for idx, video_path in enumerate(playlist, start=1):
                    if stopped:
                        break
                    next_tick = _play_one(
                        writer=writer,
                        video_path=video_path,
                        idx=idx,
                        total=len(playlist),
                        encode_params=encode_params,
                        period=period,
                        next_tick=next_tick,
                        summary=summary,
                        stopped_ref=lambda: stopped,
                    )
                if not loop or stopped:
                    break
                log.info("replay: playlist exhausted, looping (pass %d done)", pass_idx)
    finally:
        log.info(
            "replay: done — played %d video(s), %d frames, %.1f min total",
            summary.videos_played, summary.frames_published, summary.duration_s / 60,
        )

    return summary


def _play_one(
    writer: ShmFrameWriter,
    video_path: Path,
    idx: int,
    total: int,
    encode_params: list[int],
    period: float,
    next_tick: float,
    summary: ReplaySummary,
    stopped_ref,
) -> float:
    """Play a single video. Returns the updated next_tick deadline."""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        log.error("replay: cannot open video, skipping: %s", video_path)
        return next_tick

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or -1
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    needs_resize = (src_w, src_h) != (TARGET_W, TARGET_H)

    log.info(
        "replay: starting video %d/%d: %s (%dx%d, %d frames)",
        idx, total, video_path.name, src_w, src_h, total_frames,
    )

    started = time.monotonic()
    frame_idx = 0
    last_log = started
    last_log_frames = 0
    published_here = 0

    try:
        while not stopped_ref():
            ok, frame = cap.read()
            if not ok:
                break
            frame_idx += 1

            if needs_resize:
                frame = cv2.resize(frame, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)

            ok_jpg, buf = cv2.imencode(".jpg", frame, encode_params)
            if not ok_jpg:
                log.warning("replay: JPEG encode failed at %s frame %d", video_path.name, frame_idx)
                continue

            writer.write_frame(TARGET_W, TARGET_H, buf.tobytes())
            published_here += 1
            summary.frames_published += 1

            now = time.monotonic()
            if now - last_log >= 1.0:
                inst_fps = (published_here - last_log_frames) / max(now - last_log, 1e-3)
                log.info(
                    "replay: [%d/%d] frame %d/%s at %.1f fps",
                    idx, total, frame_idx,
                    total_frames if total_frames > 0 else "?", inst_fps,
                )
                last_log = now
                last_log_frames = published_here

            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    finally:
        cap.release()
        elapsed = time.monotonic() - started
        summary.duration_s += elapsed
        summary.videos_played += 1
        log.info(
            "replay: finished video %d/%d: %s — %d frames in %.1fs",
            idx, total, video_path.name, published_here, elapsed,
        )

    return next_tick
