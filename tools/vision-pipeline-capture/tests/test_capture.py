"""Tests for the downscale path of capture.py."""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from vision_pipeline_capture.capture import MAX_H, MAX_W, downscale_frame  # noqa: E402


# --------------------------------------------------------------------------- #
# Mock camera
# --------------------------------------------------------------------------- #


@dataclass
class MockCamera:
    """Minimal dxcam-like stub that always returns a fixed-size frame."""

    width: int
    height: int

    def grab(self, region: object = None) -> np.ndarray:  # noqa: ARG002
        # gradient so resize is observable (not just zeros)
        x = np.linspace(0, 255, self.width, dtype=np.uint8)
        y = np.linspace(0, 255, self.height, dtype=np.uint8)
        b = np.broadcast_to(x, (self.height, self.width)).astype(np.uint8)
        g = np.broadcast_to(y[:, None], (self.height, self.width)).astype(np.uint8)
        r = ((b.astype(int) + g.astype(int)) // 2).astype(np.uint8)
        return np.stack([b, g, r], axis=-1)


# --------------------------------------------------------------------------- #
# Pure-function tests
# --------------------------------------------------------------------------- #


def test_constants_are_pipeline_spec() -> None:
    assert (MAX_W, MAX_H) == (1920, 1080)


def test_downscale_passthrough_at_1080p() -> None:
    cam = MockCamera(1920, 1080)
    frame = cam.grab()
    out = downscale_frame(frame)
    # at-budget frames are returned unchanged (same numpy buffer)
    assert out is frame
    assert out.shape[:2] == (1080, 1920)


def test_downscale_passthrough_below_budget() -> None:
    cam = MockCamera(1280, 720)
    frame = cam.grab()
    out = downscale_frame(frame)
    assert out is frame
    assert out.shape[:2] == (720, 1280)


def test_downscale_from_1440p_mock_camera() -> None:
    """Spec: 2560x1440 mock camera frame must land at exactly 1920x1080."""
    cam = MockCamera(2560, 1440)
    frame = cam.grab()
    assert frame.shape[:2] == (1440, 2560)

    out = downscale_frame(frame)
    assert out.shape[:2] == (1080, 1920)
    assert out.dtype == np.uint8
    assert out.shape[2] == 3  # BGR channels preserved


def test_downscale_from_4k_mock_camera() -> None:
    cam = MockCamera(3840, 2160)
    frame = cam.grab()
    out = downscale_frame(frame)
    assert out.shape[:2] == (1080, 1920)


def test_downscale_custom_ceiling() -> None:
    cam = MockCamera(2560, 1440)
    frame = cam.grab()
    out = downscale_frame(frame, max_w=1280, max_h=720)
    assert out.shape[:2] == (720, 1280)


def test_log_emitted_once_across_many_calls(caplog: pytest.LogCaptureFixture) -> None:
    cam = MockCamera(2560, 1440)
    state: dict[str, bool] = {}
    with caplog.at_level(logging.INFO, logger="vision_pipeline_capture.capture"):
        for _ in range(5):
            downscale_frame(cam.grab(), _logged=state)
    msgs = [r.getMessage() for r in caplog.records if "downscaling" in r.getMessage()]
    assert len(msgs) == 1, f"expected single downscale log line, got {len(msgs)}: {msgs}"
    assert "2560x1440" in msgs[0] and "1920x1080" in msgs[0]


def test_no_log_when_state_dict_missing() -> None:
    """Calling without _logged dict must not raise (helper is still pure)."""
    cam = MockCamera(2560, 1440)
    out = downscale_frame(cam.grab())  # no _logged provided
    assert out.shape[:2] == (1080, 1920)
