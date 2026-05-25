"""VMM-2 — Minimap sidecar main loop.

Runs at DEFAULT_FPS (10 Hz), captures ETS2 window, detects the red route line
on the minimap via OpenCV, and writes detected pixel-space points to the
TruckPilotMinimapLine SHM region consumed by the minimap-vision Rust plugin.

Graceful shutdown:
  - SIGINT / SIGTERM / Ctrl+C: exits cleanly, SHM is closed.
  - ETS2 not running on startup: exits with clear error message (exit code 1).
  - ETS2 crashes mid-run: logs repeated grab failures, keeps looping until
    REMAP_RETRY_FRAMES failures in a row, then exits (exit code 2).
  - [vision.minimap] missing from TOML: exits with error message (exit code 3).
"""

from __future__ import annotations

import logging
import signal
import sys
import time
from pathlib import Path

from . import DEFAULT_FPS, DEFAULT_SHM_NAME
from .capture import ScreenCapture
from .config import DEFAULT_TOML, load_config
from .pipeline import detect_route_line
from .shm_writer import MinimapShmWriter

log = logging.getLogger(__name__)

# After this many consecutive grab failures, give up.
MAX_CONSECUTIVE_FAILURES = 100


def run_sidecar(
    toml_path: Path = DEFAULT_TOML,
    fps: int = DEFAULT_FPS,
    shm_name: str = DEFAULT_SHM_NAME,
    window_title: str = "Euro Truck Simulator 2",
    debug_preview: bool = False,
) -> int:
    """Main loop. Returns process exit code."""

    # ── load config ───────────────────────────────────────────────────────────
    try:
        cfg = load_config(toml_path)
    except (FileNotFoundError, KeyError) as exc:
        log.error("%s", exc)
        return 3

    log.info(
        "minimap sidecar starting — ROI (%d,%d) %dx%d  HSV [%s..%s]  fps=%d shm='%s'",
        cfg.x, cfg.y, cfg.w, cfg.h,
        cfg.hsv_lower, cfg.hsv_upper,
        fps, shm_name,
    )

    # ── init capture ──────────────────────────────────────────────────────────
    try:
        cap = ScreenCapture(window_title)
    except RuntimeError as exc:
        log.error("capture init failed: %s", exc)
        return 1

    # Warm-up grab to verify ETS2 is visible.
    test_frame = cap.grab()
    if test_frame is None:
        log.error(
            "ETS2 window '%s' returned no frame. "
            "Make sure the game is running and visible (not minimised).",
            window_title,
        )
        cap.release()
        return 1

    log.info("first frame captured: %dx%d", test_frame.shape[1], test_frame.shape[0])

    # ── SHM writer ────────────────────────────────────────────────────────────
    with MinimapShmWriter(shm_name) as writer:

        # ── signal handling ───────────────────────────────────────────────────
        quit_flag = {"v": False}

        def _on_signal(sig: int, _: object) -> None:
            log.info("received signal %d — shutting down", sig)
            quit_flag["v"] = True

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)

        # ── main loop ─────────────────────────────────────────────────────────
        period = 1.0 / fps
        next_tick = time.monotonic()
        consecutive_failures = 0
        frames_published = 0
        frames_skipped = 0

        while not quit_flag["v"]:
            frame = cap.grab()

            if frame is None:
                consecutive_failures += 1
                frames_skipped += 1
                if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                    log.error(
                        "%d consecutive frame grabs returned None — "
                        "ETS2 may have crashed or been minimised. Exiting.",
                        consecutive_failures,
                    )
                    cap.release()
                    return 2
                # Sleep for one tick before retry.
                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.monotonic()
                continue

            consecutive_failures = 0

            result = detect_route_line(
                frame,
                cfg.x, cfg.y, cfg.w, cfg.h,
                cfg.hsv_lower, cfg.hsv_upper,
            )

            writer.write_points(result.points, result.confidence)
            frames_published += 1

            log.debug(
                "frame=%d pts=%d conf=%.2f mask_px=%d skel_px=%d",
                frames_published,
                len(result.points),
                result.confidence,
                result.mask_pixel_count,
                result.skeleton_pixel_count,
            )

            if debug_preview:
                _show_debug_preview(frame, cfg, result)

            next_tick += period
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()

        cap.release()

    log.info(
        "sidecar stopped — published=%d skipped=%d",
        frames_published, frames_skipped,
    )
    return 0


def _show_debug_preview(frame, cfg, result) -> None:
    """Optional OpenCV debug window (--debug flag). Shows mask + detected points."""
    import cv2
    import numpy as np

    x, y, w, h = cfg.x, cfg.y, cfg.w, cfg.h
    fh, fw = frame.shape[:2]
    x2, y2 = min(fw, x + w), min(fh, y + h)
    roi = frame[max(0, y):y2, max(0, x):x2].copy()

    # Draw detected points.
    for px, py in result.points:
        cv2.circle(roi, (int(px), int(py)), 2, (0, 255, 0), -1)

    # Status overlay.
    cv2.putText(
        roi,
        f"pts={len(result.points)} conf={result.confidence:.2f}",
        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
    )
    cv2.imshow("Minimap Sidecar — Q to quit", roi)
    if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
        import os, signal as _sig
        os.kill(os.getpid(), _sig.SIGTERM)
