"""DXcam ETS2 window capture helper (mirrors vision-pipeline-capture pattern)."""

from __future__ import annotations

import logging
import sys

import numpy as np

log = logging.getLogger(__name__)


def _find_window_rect(title: str) -> tuple[int, int, int, int] | None:
    if sys.platform != "win32":
        return None
    try:
        import win32gui
    except ImportError:
        log.warning("pywin32 not installed; full-screen capture fallback")
        return None

    title_l = title.lower()
    matches: list[int] = []

    def _cb(hwnd: int, _: object) -> bool:
        if win32gui.IsWindowVisible(hwnd):
            text = win32gui.GetWindowText(hwnd)
            if text and title_l in text.lower():
                matches.append(hwnd)
        return True

    win32gui.EnumWindows(_cb, None)
    if not matches:
        return None
    hwnd = matches[0]
    rect = win32gui.GetClientRect(hwnd)
    lt = win32gui.ClientToScreen(hwnd, (0, 0))
    rb = win32gui.ClientToScreen(hwnd, (rect[2], rect[3]))
    return (lt[0], lt[1], rb[0], rb[1])


class ScreenCapture:
    """Thin wrapper around dxcam for repeated grabs at a fixed region."""

    def __init__(self, window_title: str = "Euro Truck Simulator 2") -> None:
        try:
            import dxcam  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "dxcam not installed (Windows only). Install with: pip install dxcam"
            ) from exc

        self._region = _find_window_rect(window_title)
        if self._region is None:
            log.warning("ETS2 window '%s' not found — capturing full primary display", window_title)

        self._cam = dxcam.create(output_color="BGR")
        if self._cam is None:
            raise RuntimeError("dxcam.create() returned None — no DXGI device")

    def grab(self) -> np.ndarray | None:
        """Return one BGR frame or None if dxcam returned nothing."""
        return self._cam.grab(region=self._region) if self._region else self._cam.grab()

    def release(self) -> None:
        try:
            self._cam.release()
        except Exception:
            pass
