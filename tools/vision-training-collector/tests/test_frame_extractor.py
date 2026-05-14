"""Test frame extractor against a synthetic 10-frame video."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from vision_training_collector.config import Config  # noqa: E402
from vision_training_collector.frame_extractor import extract_video  # noqa: E402


def _make_video(path: Path, n_frames: int = 10, fps: int = 5) -> None:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, float(fps), (320, 180))
    assert writer.isOpened(), "OpenCV cannot write mp4v - install proper codec"
    rng = np.random.default_rng(0)
    for i in range(n_frames):
        # alternating: colorful + dark frames so skip heuristics matter
        if i % 2 == 0:
            frame = rng.integers(0, 255, (180, 320, 3), dtype=np.uint8)
        else:
            frame = np.full((180, 320, 3), 5, dtype=np.uint8)  # near-black
        writer.write(frame)
    writer.release()


def test_extract_video_writes_frames(tmp_path: Path) -> None:
    video = tmp_path / "synthetic.mp4"
    _make_video(video, n_frames=10, fps=5)
    out_dir = tmp_path / "frames"

    cfg = Config()
    cfg.extract.frame_interval_seconds = 0.2  # every 1 frame at fps=5
    cfg.extract.target_width = 320
    cfg.extract.target_height = 180
    cfg.extract.skip_menu_color_count = 5  # synthetic noise frames will pass

    stats = extract_video(video, out_dir, cfg)
    assert stats["read"] == 10
    assert stats["saved"] >= 1
    assert stats["skipped_black"] >= 1
    written = sorted(out_dir.glob("frame_*.jpg"))
    assert len(written) == stats["saved"]
