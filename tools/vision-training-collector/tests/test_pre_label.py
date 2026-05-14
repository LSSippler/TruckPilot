"""Pre-label pipeline smoke test with mocked inference."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

from vision_training_collector import pre_label as pl
from vision_training_collector.pre_label import ClassMapping, pre_label_directory


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


def test_pre_label_pipeline_three_tiers(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    in_dir.mkdir()

    img_a = in_dir / "a.jpg"; _make_img(img_a, (255, 0, 0))
    img_b = in_dir / "b.jpg"; _make_img(img_b, (0, 255, 0))
    img_c = in_dir / "c.jpg"; _make_img(img_c, (0, 0, 255))

    # Mock the model loader so we don't need ETS2LA weights.
    monkeypatch.setattr(pl, "_load_model", lambda model_path: _MockModel())

    # Per-image canned detections covering all three tiers + drops.
    # Format: (ets2la_id, conf, cx, cy, w, h) normalized 0-1.
    fake_dets: dict[str, list[tuple[int, float, float, float, float, float]]] = {
        "a.jpg": [
            (0, 0.95, 0.5, 0.5, 0.1, 0.1),   # car, auto
            (1, 0.50, 0.2, 0.2, 0.1, 0.1),   # truck, review
            (2, 0.90, 0.3, 0.3, 0.1, 0.1),   # van, suppressed -> dropped_unmapped
        ],
        "b.jpg": [
            (17, 0.95, 0.5, 0.5, 0.05, 0.05),  # red light, auto
        ],
        "c.jpg": [],  # no detections -> images_without_detections
    }

    def _fake_infer(model: Any, image_path: Path, conf_min: float):
        return [d for d in fake_dets.get(image_path.name, []) if d[1] >= conf_min]

    monkeypatch.setattr(pl, "_infer", _fake_infer)

    mapping = ClassMapping.from_yaml(_project_root() / "class_mapping.yaml")
    fake_model_path = tmp_path / "fake.pt"
    fake_model_path.write_bytes(b"")  # exists so FileNotFoundError check passes

    report = pre_label_directory(in_dir, out_dir, mapping, fake_model_path)

    assert report["input_images"] == 3
    assert report["processed"] == 3
    assert report["skipped"] == 0
    assert report["detections_per_tier"]["auto_accept"] == 2   # a:car, b:red
    assert report["detections_per_tier"]["review"] == 1        # a:truck
    assert report["detections_per_tier"]["dropped_unmapped"] == 1
    assert report["images_without_detections"] == 1
    assert report["detections_per_class"]["Car"] == 1
    assert report["detections_per_class"]["Truck"] == 1
    assert report["detections_per_class"]["TrafficLightRed"] == 1

    # Output structure
    assert (out_dir / "images" / "a.jpg").exists() or (out_dir / "images" / "a.jpg").is_symlink()
    assert (out_dir / "labels" / "auto" / "a.txt").exists()
    assert (out_dir / "labels" / "review" / "a.txt").exists()
    assert (out_dir / "labels" / "auto" / "b.txt").exists()
    assert (out_dir / "labels" / "manual" / "c.txt").exists()
    assert (out_dir / "pre_label_report.json").exists()

    # YOLO format: first token is class id
    auto_line = (out_dir / "labels" / "auto" / "a.txt").read_text().strip().split()
    assert auto_line[0] == "0"  # Car
    assert len(auto_line) == 5  # cls cx cy w h
