"""Tests for the export label-copy + warn behavior."""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from vision_training_collector.config import Config  # noqa: E402
from vision_training_collector.exporter import _find_label_for, export  # noqa: E402


def _write_img(path: Path, color: tuple[int, int, int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    img = np.full((90, 160, 3), color, dtype=np.uint8)
    cv2.imwrite(str(path), img)


# --------------------------------------------------------------------- find-label heuristics


def test_find_label_sibling_wins(tmp_path: Path) -> None:
    src_root = tmp_path / "deduped"
    img = src_root / "vid1" / "frame_000000.jpg"
    _write_img(img, (10, 10, 10))
    sib = img.with_suffix(".txt")
    sib.write_text("0 0.5 0.5 0.1 0.1\n", encoding="utf-8")
    out = _find_label_for(img, src_root)
    assert out == sib


def test_find_label_parallel_labels_dir(tmp_path: Path) -> None:
    src_root = tmp_path / "deduped"
    img = src_root / "vid1" / "frame_000000.jpg"
    _write_img(img, (10, 10, 10))
    parallel = src_root / "labels" / "vid1" / "frame_000000.txt"
    parallel.parent.mkdir(parents=True, exist_ok=True)
    parallel.write_text("1 0.5 0.5 0.1 0.1\n", encoding="utf-8")
    out = _find_label_for(img, src_root)
    assert out == parallel


def test_find_label_returns_none_when_absent(tmp_path: Path) -> None:
    src_root = tmp_path / "deduped"
    img = src_root / "vid1" / "frame_000000.jpg"
    _write_img(img, (10, 10, 10))
    assert _find_label_for(img, src_root) is None


# --------------------------------------------------------------------- end-to-end export


def test_export_copies_matching_labels_and_warns_on_missing(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    src = tmp_path / "deduped"
    final = tmp_path / "final"

    # 10 images so the 80/15/5 split lands at 8/1/1.
    for i in range(10):
        img = src / "vidA" / f"frame_{i:06d}.jpg"
        _write_img(img, (i * 25, 0, 255 - i * 25))
        # Label exists for every other image.
        if i % 2 == 0:
            img.with_suffix(".txt").write_text(f"{i % 3} 0.5 0.5 0.2 0.2\n", encoding="utf-8")

    cfg = Config()  # default 0.80/0.15/0.05 + seed 42

    with caplog.at_level(logging.WARNING, logger="vision_training_collector.exporter"):
        stats = export(src, final, cfg)

    assert stats["total"] == 10
    assert (stats["train"], stats["val"], stats["test"]) == (8, 1, 1)
    assert stats["labels_copied"] == 5
    assert stats["labels_missing"] == 5

    # the warning must appear at least once
    missing_warnings = [
        r for r in caplog.records
        if r.levelno == logging.WARNING and "no label found" in r.getMessage()
    ]
    assert len(missing_warnings) >= 1

    # at least one .txt should live under final/labels/<split>/ now
    copied_labels = list((final / "labels").rglob("*.txt"))
    assert len(copied_labels) == 5
    # each copied label sits next to its image by stem under matching split
    for lbl in copied_labels:
        split = lbl.parent.name
        stem = lbl.stem
        assert (final / "images" / split / f"{stem}.jpg").exists()
