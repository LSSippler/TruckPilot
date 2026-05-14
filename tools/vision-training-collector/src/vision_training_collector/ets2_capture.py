"""ETS2 live frame capture via dxcam + hotkey trigger."""

from __future__ import annotations

import logging
import sys
import time
from datetime import datetime
from pathlib import Path

from .config import Config

log = logging.getLogger(__name__)


def _import_dxcam():
    try:
        import dxcam  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("dxcam not installed (Windows only).") from exc
    return dxcam


def _import_keyboard():
    try:
        import keyboard  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("keyboard module not installed (required for hotkeys).") from exc
    return keyboard


def capture_live(cfg: Config, out_dir: Path, auto: bool = False) -> int:
    """Capture frames from primary display. F8=save current frame, F9=quit.

    When auto=True, saves a frame every cfg.capture.auto_interval_seconds.
    Returns the number of frames saved.
    """
    if sys.platform != "win32":
        raise RuntimeError("ETS2 live capture is Windows-only.")

    out_dir.mkdir(parents=True, exist_ok=True)
    dxcam = _import_dxcam()
    keyboard = _import_keyboard()

    camera = dxcam.create(output_color="BGR")
    if camera is None:
        raise RuntimeError("dxcam.create() returned None - no display?")

    saved = 0
    quit_flag = {"v": False}
    save_flag = {"v": False}

    def _on_save() -> None:
        save_flag["v"] = True

    def _on_quit() -> None:
        quit_flag["v"] = True

    keyboard.add_hotkey(cfg.capture.hotkey_save, _on_save)
    keyboard.add_hotkey(cfg.capture.hotkey_quit, _on_quit)
    log.info(
        "Live capture active. %s=save frame, %s=quit. Auto=%s (interval %.1fs)",
        cfg.capture.hotkey_save,
        cfg.capture.hotkey_quit,
        auto,
        cfg.capture.auto_interval_seconds,
    )

    import cv2

    last_auto = 0.0
    try:
        while not quit_flag["v"]:
            now = time.time()
            do_save = False
            if save_flag["v"]:
                save_flag["v"] = False
                do_save = True
            elif auto and (now - last_auto) >= cfg.capture.auto_interval_seconds:
                do_save = True
                last_auto = now

            if do_save:
                frame = camera.grab()
                if frame is None:
                    time.sleep(0.05)
                    continue
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                path = out_dir / f"live_{ts}.png"
                cv2.imwrite(str(path), frame)
                saved += 1
                log.info("saved %s (total=%d)", path.name, saved)
            else:
                time.sleep(0.05)
    finally:
        try:
            keyboard.remove_hotkey(cfg.capture.hotkey_save)
            keyboard.remove_hotkey(cfg.capture.hotkey_quit)
        except Exception:  # noqa: BLE001
            pass
        try:
            camera.release()
        except Exception:  # noqa: BLE001
            pass

    log.info("Capture finished. Saved %d frames to %s", saved, out_dir)
    return saved
