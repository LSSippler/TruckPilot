"""DXcam-based ETS2 window capture loop with JPEG encode + SHM publish."""

from __future__ import annotations

import logging
import sys
import time
from dataclasses import dataclass
from typing import Callable

from . import DEFAULT_FPS, DEFAULT_JPEG_QUALITY, DEFAULT_SHM_NAME
from .shm_writer import ShmFrameWriter

log = logging.getLogger(__name__)


@dataclass
class CaptureStats:
    frames_published: int = 0
    frames_skipped: int = 0
    last_fps: float = 0.0
    last_jpeg_bytes: int = 0
    started_at: float = 0.0
    paused: bool = False


def _import_dxcam():
    try:
        import dxcam  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("dxcam not installed (Windows only).") from exc
    return dxcam


def _import_cv2():
    try:
        import cv2  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("opencv-python not installed.") from exc
    return cv2


def _import_keyboard():
    try:
        import keyboard  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("keyboard not installed.") from exc
    return keyboard


def _find_window_rect(title_substring: str) -> tuple[int, int, int, int] | None:
    """Return (left, top, right, bottom) for the first visible window matching title."""
    if sys.platform != "win32":
        return None
    try:
        import win32gui  # type: ignore[import-not-found]
    except ImportError:
        log.warning("pywin32 not installed; cannot bound capture to window. Full-screen capture.")
        return None

    target = title_substring.lower()
    matches: list[int] = []

    def _cb(hwnd: int, _: object) -> bool:
        if not win32gui.IsWindowVisible(hwnd):
            return True
        text = win32gui.GetWindowText(hwnd)
        if text and target in text.lower():
            matches.append(hwnd)
        return True

    win32gui.EnumWindows(_cb, None)
    if not matches:
        return None
    hwnd = matches[0]
    rect = win32gui.GetClientRect(hwnd)
    # client coords are 0,0,w,h - translate to screen
    left_top = win32gui.ClientToScreen(hwnd, (0, 0))
    right_bot = win32gui.ClientToScreen(hwnd, (rect[2], rect[3]))
    return (left_top[0], left_top[1], right_bot[0], right_bot[1])


# --------------------------------------------------------------------- main loop


def run_capture(
    shm_name: str = DEFAULT_SHM_NAME,
    fps: int = DEFAULT_FPS,
    jpeg_quality: int = DEFAULT_JPEG_QUALITY,
    window_title: str = "Euro Truck Simulator 2",
    pause_hotkey: str = "F8",
    quit_hotkey: str = "F9",
    on_stats: Callable[[CaptureStats], None] | None = None,
) -> CaptureStats:
    """Capture ETS2 window at `fps`, JPEG-encode, publish to SHM. F8=pause, F9=quit."""
    if sys.platform != "win32":
        raise RuntimeError("Vision-pipeline-capture is Windows-only.")

    dxcam = _import_dxcam()
    cv2 = _import_cv2()
    keyboard = _import_keyboard()

    region = _find_window_rect(window_title)
    if region is None:
        log.warning("window '%s' not found - capturing full primary display", window_title)

    camera = dxcam.create(output_color="BGR")
    if camera is None:
        raise RuntimeError("dxcam.create() returned None - no DXGI device available")

    stats = CaptureStats(started_at=time.monotonic())
    quit_flag = {"v": False}

    def _toggle_pause() -> None:
        stats.paused = not stats.paused
        log.info("capture %s", "PAUSED" if stats.paused else "RESUMED")

    def _quit() -> None:
        quit_flag["v"] = True

    keyboard.add_hotkey(pause_hotkey, _toggle_pause)
    keyboard.add_hotkey(quit_hotkey, _quit)
    log.info("capture starting: shm=%s fps=%d region=%s quality=%d", shm_name, fps, region, jpeg_quality)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
    period = 1.0 / float(fps)
    next_tick = time.monotonic()
    last_window = time.monotonic()
    frames_in_window = 0

    try:
        with ShmFrameWriter(name=shm_name) as writer:
            while not quit_flag["v"]:
                if stats.paused:
                    time.sleep(0.05)
                    next_tick = time.monotonic()  # reset drift tracker
                    continue

                frame = camera.grab(region=region) if region else camera.grab()
                if frame is None:
                    stats.frames_skipped += 1
                else:
                    h, w = frame.shape[:2]
                    ok, buf = cv2.imencode(".jpg", frame, encode_params)
                    if not ok:
                        stats.frames_skipped += 1
                    else:
                        jpeg_bytes = buf.tobytes()
                        ts_us = int(time.monotonic() * 1_000_000)
                        writer.write_frame(width=w, height=h, jpeg_bytes=jpeg_bytes, timestamp_us=ts_us)
                        stats.frames_published += 1
                        stats.last_jpeg_bytes = len(jpeg_bytes)
                        frames_in_window += 1

                # drift-corrected sleep
                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    # we overran the deadline; resync to now so we don't burn-spin
                    next_tick = time.monotonic()

                now = time.monotonic()
                if now - last_window >= 1.0:
                    stats.last_fps = frames_in_window / (now - last_window)
                    frames_in_window = 0
                    last_window = now
                    if on_stats:
                        on_stats(stats)
                    log.debug(
                        "fps=%.1f published=%d skipped=%d jpeg=%dKB",
                        stats.last_fps,
                        stats.frames_published,
                        stats.frames_skipped,
                        stats.last_jpeg_bytes // 1024,
                    )
    finally:
        try:
            keyboard.remove_hotkey(pause_hotkey)
            keyboard.remove_hotkey(quit_hotkey)
        except Exception:  # noqa: BLE001
            pass
        try:
            camera.release()
        except Exception:  # noqa: BLE001
            pass

    log.info(
        "capture finished: published=%d skipped=%d duration=%.1fs",
        stats.frames_published,
        stats.frames_skipped,
        time.monotonic() - stats.started_at,
    )
    return stats
