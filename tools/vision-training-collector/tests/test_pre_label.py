"""Pre-label pipeline smoke test with mocked inference."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

from vision_training_collector import pre_label as pl  # noqa: E402
from vision_training_collector.pre_label import ClassMapping, pre_label_directory  # noqa: E402


# --------------------------------------------------------------------------- #
# Class-mapping unit tests
# --------------------------------------------------------------------------- #


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_class_mapping_yaml_loads() -> None:
    mapping = ClassMapping.from_yaml(_project_root() / "class_mapping.yaml")
    # spec: 9 ETS2LA classes mapped onto 9 TruckPilot classes
    assert len(mapping.ets2la_to_truckpilot) == 9
    # explicit mappings from the spec
    assert mapping.ets2la_to_truckpilot[0] == 0   # car -> Car
    assert mapping.ets2la_to_truckpilot[1] == 1   # truck -> Truck
    assert mapping.ets2la_to_truckpilot[3] == 3   # bus -> Bus
    assert mapping.ets2la_to_truckpilot[4] == 10  # stop_sign -> StopSign
    assert mapping.ets2la_to_truckpilot[6] == 11  # speedlimit_sign -> SpeedLimitSign
    assert mapping.ets2la_to_truckpilot[15] == 9  # green -> TrafficLightGreen
    assert mapping.ets2la_to_truckpilot[16] == 8  # yellow -> TrafficLightYellow
    assert mapping.ets2la_to_truckpilot[17] == 7  # red -> TrafficLightRed
    assert mapping.ets2la_to_truckpilot[21] == 12 # lane_separator -> LaneSolid
    # suppressed classes return None
    assert mapping.map_class(2) is None   # van suppressed
    assert mapping.map_class(14) is None  # traffic_cone suppressed
    # unmapped (not suppressed, not mapped) also returns None
    assert mapping.map_class(99) is None
    # manual-only TruckPilot ids
    assert {2, 4, 5, 6, 13, 14} == mapping.manual_only_classes


def test_confidence_tier_thresholds() -> None:
    mapping = ClassMapping.from_yaml(_project_root() / "class_mapping.yaml")
    assert mapping.auto_accept == 0.85
    assert mapping.review_min == 0.30


# --------------------------------------------------------------------------- #
# Pipeline smoke test (mocked model)
# --------------------------------------------------------------------------- #


class _MockModel:
    """Mimics ultralytics.YOLO enough for the inference adapter — but lives in a
    non-ultralytics module, so we bypass _infer's adapter and patch _infer instead.
    """


def _make_img(path: Path, color: tuple[int, int, int]) -> None:
    img = np.full((180, 320, 3), color, dtype=np.uint8)
    cv2.imwrite(str(path), img)


def _patch_model(monkeypatch: pytest.MonkeyPatch, dets_by_name: dict[str, list[tuple]]) -> None:
    monkeypatch.setattr(pl, "_load_model", lambda model_path: _MockModel())

    def _fake_infer(model: Any, image_path: Path, conf_min: float):
        return [d for d in dets_by_name.get(image_path.name, []) if d[1] >= conf_min]

    monkeypatch.setattr(pl, "_infer", _fake_infer)


def _read_manifest_rows(out_dir: Path) -> list[dict[str, str]]:
    import csv
    with (out_dir / "pre_label_manifest.csv").open(encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def test_pre_label_split_aware_layout(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    # Standard YOLO split layout
    (in_dir / "train").mkdir(parents=True)
    (in_dir / "val").mkdir(parents=True)
    (in_dir / "test").mkdir(parents=True)

    img_a = in_dir / "train" / "a.jpg"
    _make_img(img_a, (255, 0, 0))
    img_b = in_dir / "train" / "b.jpg"
    _make_img(img_b, (0, 255, 0))
    img_v = in_dir / "val" / "v.jpg"
    _make_img(img_v, (0, 0, 255))
    img_t = in_dir / "test" / "t.jpg"
    _make_img(img_t, (128, 128, 128))  # no detections

    _patch_model(
        monkeypatch,
        {
            "a.jpg": [
                (0, 0.95, 0.5, 0.5, 0.1, 0.1),   # car, auto
                (1, 0.50, 0.2, 0.2, 0.1, 0.1),   # truck, review -> demotes a to "review"
                (2, 0.90, 0.3, 0.3, 0.1, 0.1),   # van, suppressed
            ],
            "b.jpg": [
                (17, 0.95, 0.5, 0.5, 0.05, 0.05),  # red light, auto -> tier "auto"
            ],
            "v.jpg": [
                (0, 0.45, 0.5, 0.5, 0.1, 0.1),   # car, review -> tier "review"
            ],
            "t.jpg": [],  # no detections -> tier "none"
        },
    )

    mapping = ClassMapping.from_yaml(_project_root() / "class_mapping.yaml")
    fake_model_path = tmp_path / "fake.pt"
    fake_model_path.write_bytes(b"")

    report = pre_label_directory(in_dir, out_dir, mapping, fake_model_path)

    assert report["input_images"] == 4
    assert report["processed"] == 4
    assert report["splits"] == {"train": 2, "val": 1, "test": 1}
    assert report["detections_per_tier"]["auto_accept"] == 2
    assert report["detections_per_tier"]["review"] == 2
    assert report["detections_per_tier"]["dropped_unmapped"] == 1
    assert report["images_without_detections"] == 1

    # Images mirrored under <split>/
    assert (out_dir / "images" / "train" / "a.jpg").exists() or (out_dir / "images" / "train" / "a.jpg").is_symlink()
    assert (out_dir / "images" / "train" / "b.jpg").exists() or (out_dir / "images" / "train" / "b.jpg").is_symlink()
    assert (out_dir / "images" / "val" / "v.jpg").exists() or (out_dir / "images" / "val" / "v.jpg").is_symlink()
    assert (out_dir / "images" / "test" / "t.jpg").exists() or (out_dir / "images" / "test" / "t.jpg").is_symlink()

    # Labels: a (review demotion) + b (auto) + v (review). t has none -> no file.
    assert (out_dir / "labels" / "train" / "a.txt").exists()
    assert (out_dir / "labels" / "train" / "b.txt").exists()
    assert (out_dir / "labels" / "val" / "v.txt").exists()
    assert not (out_dir / "labels" / "test" / "t.txt").exists()

    # Both auto and review detections for `a` end up in the same file
    a_lines = (out_dir / "labels" / "train" / "a.txt").read_text().strip().splitlines()
    assert len(a_lines) == 2
    assert {ln.split()[0] for ln in a_lines} == {"0", "1"}

    # Manifest rows
    rows = _read_manifest_rows(out_dir)
    by_file = {r["filename"]: r for r in rows}
    assert set(by_file) == {
        "images/train/a.jpg",
        "images/train/b.jpg",
        "images/val/v.jpg",
        "images/test/t.jpg",
    }
    assert by_file["images/train/a.jpg"]["tier"] == "review"  # demoted: has a review-tier det
    assert by_file["images/train/a.jpg"]["split"] == "train"
    assert int(by_file["images/train/a.jpg"]["num_detections"]) == 2
    assert float(by_file["images/train/a.jpg"]["confidence_max"]) == pytest.approx(0.95)
    assert by_file["images/train/b.jpg"]["tier"] == "auto"
    assert by_file["images/val/v.jpg"]["tier"] == "review"
    assert by_file["images/test/t.jpg"]["tier"] == "none"
    assert int(by_file["images/test/t.jpg"]["num_detections"]) == 0


def test_pre_label_flat_layout_keeps_legacy_behavior(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    in_dir.mkdir()
    img = in_dir / "x.jpg"
    _make_img(img, (0, 128, 200))

    _patch_model(monkeypatch, {"x.jpg": [(0, 0.95, 0.5, 0.5, 0.1, 0.1)]})
    mapping = ClassMapping.from_yaml(_project_root() / "class_mapping.yaml")
    fake_model_path = tmp_path / "fake.pt"
    fake_model_path.write_bytes(b"")

    report = pre_label_directory(in_dir, out_dir, mapping, fake_model_path)

    assert report["splits"] == {"flat": 1}
    assert (out_dir / "images" / "x.jpg").exists() or (out_dir / "images" / "x.jpg").is_symlink()
    assert (out_dir / "labels" / "x.txt").exists()

    rows = _read_manifest_rows(out_dir)
    assert len(rows) == 1
    assert rows[0]["split"] == "flat"
    assert rows[0]["filename"] == "images/x.jpg"
    assert rows[0]["tier"] == "auto"
    assert int(rows[0]["num_detections"]) == 1
