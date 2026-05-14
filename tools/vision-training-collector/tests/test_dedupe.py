"""Test perceptual-hash dedupe with synthetic images."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("PIL")
pytest.importorskip("imagehash")

from vision_training_collector.config import Config
from vision_training_collector.deduplicator import dedupe_dir


def _write_image(path: Path, color: tuple[int, int, int], noise: int = 0) -> None:
    img = np.full((128, 128, 3), color, dtype=np.uint8)
    if noise:
        rng = np.random.default_rng(0)
        img = np.clip(img.astype(int) + rng.integers(-noise, noise, img.shape), 0, 255).astype(np.uint8)
    cv2.imwrite(str(path), img)


def test_dedupe_keeps_distinct_drops_duplicates(tmp_path: Path) -> None:
    src = tmp_path / "src"
    dst = tmp_path / "dst"
    src.mkdir()

    _write_image(src / "red_a.png", (0, 0, 255))
    _write_image(src / "red_b.png", (0, 0, 255), noise=2)  # near-duplicate
    _write_image(src / "green.png", (0, 255, 0))
    _write_image(src / "blue.png", (255, 0, 0))

    cfg = Config()
    cfg.dedupe.phash_threshold = 5
    stats = dedupe_dir(src, dst, cfg)

    assert stats["input"] == 4
    assert stats["kept"] >= 3  # red duplicates collapsed
    assert stats["dropped"] >= 1
    assert stats["kept"] + stats["dropped"] + stats["errors"] == stats["input"]
