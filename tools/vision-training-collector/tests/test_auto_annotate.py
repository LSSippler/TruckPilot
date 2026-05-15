"""Unit tests for the multi-model auto-annotation pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("yaml")

from vision_training_collector import auto_annotate as aa  # noqa: E402
from vision_training_collector.auto_annotate import (  # noqa: E402
    AutoAnnotateConfig,
    Detection,
    auto_annotate_directory,
    merge_detections,
)


# --------------------------------------------------------------------------- #
# Config loader
# --------------------------------------------------------------------------- #


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_config_yaml_loads_repo_config() -> None:
    cfg = AutoAnnotateConfig.from_yaml(_project_root() / "auto_annotate_config.yaml")
    # Both models declared
    assert len(cfg.models) == 2
    by_name = {m.name: m for m in cfg.models}
    assert by_name["truckpilot_v2"].priority > by_name["yolov8x_coco"].priority
    # TruckPilot class space mirrored
    assert cfg.truckpilot_classes[0] == "Car"
    assert cfg.truckpilot_classes[10] == "StopSign"
    # COCO mapping translates to TruckPilot names
    assert by_name["yolov8x_coco"].class_mapping == {
        "car": "Car",
        "truck": "Truck",
        "bus": "Bus",
        "stop sign": "StopSign",
        "traffic light": "TrafficLightRed",
    }


def test_config_rejects_model_with_both_class_specs(tmp_path: Path) -> None:
    cfg_path = tmp_path / "bad.yaml"
    cfg_path.write_text(
        """
truckpilot_classes:
  0: Car
models:
  - name: bad
    path: x.pt
    classes: [Car]
    class_mapping: {car: Car}
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="either 'classes' or 'class_mapping'"):
        AutoAnnotateConfig.from_yaml(cfg_path)


def test_config_rejects_model_with_no_class_spec(tmp_path: Path) -> None:
    cfg_path = tmp_path / "bad.yaml"
    cfg_path.write_text(
        """
truckpilot_classes:
  0: Car
models:
  - name: bad
    path: x.pt
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="must declare 'classes' or 'class_mapping'"):
        AutoAnnotateConfig.from_yaml(cfg_path)


# --------------------------------------------------------------------------- #
# NMS merge logic
# --------------------------------------------------------------------------- #


def _det(
    tp_id: int,
    conf: float,
    cx: float,
    cy: float,
    w: float = 0.1,
    h: float = 0.1,
    *,
    model: str = "m",
    priority: int = 0,
) -> Detection:
    return Detection(
        tp_class_id=tp_id, conf=conf, cx=cx, cy=cy, w=w, h=h, model_name=model, priority=priority
    )


def test_merge_keeps_non_overlapping_detections() -> None:
    a = _det(0, 0.9, 0.2, 0.2)
    b = _det(1, 0.9, 0.8, 0.8)
    kept, conflicts = merge_detections([a, b], iou_threshold=0.5)
    assert {(d.tp_class_id, d.cx) for d in kept} == {(0, 0.2), (1, 0.8)}
    assert conflicts == 0


def test_merge_suppresses_lower_priority_on_overlap_same_class() -> None:
    hi = _det(0, 0.6, 0.5, 0.5, 0.2, 0.2, model="v2", priority=100)
    lo = _det(0, 0.95, 0.5, 0.5, 0.2, 0.2, model="coco", priority=50)
    kept, conflicts = merge_detections([hi, lo], iou_threshold=0.5)
    assert len(kept) == 1
    # Higher priority strictly wins even with lower confidence.
    assert kept[0].model_name == "v2"
    # Same class -> not counted as conflict.
    assert conflicts == 0


def test_merge_priority_wins_on_class_conflict() -> None:
    """High-priority Car overlaps low-priority Truck -> Car wins, conflict tracked."""
    car = _det(0, 0.7, 0.5, 0.5, 0.2, 0.2, model="v2", priority=100)
    truck = _det(1, 0.95, 0.5, 0.5, 0.2, 0.2, model="coco", priority=50)
    kept, conflicts = merge_detections([car, truck], iou_threshold=0.5)
    assert len(kept) == 1
    assert kept[0].tp_class_id == 0
    assert conflicts == 1


def test_merge_ties_break_on_confidence() -> None:
    """Same priority -> higher confidence wins."""
    a = _det(0, 0.6, 0.5, 0.5, 0.2, 0.2, model="m1", priority=50)
    b = _det(1, 0.9, 0.5, 0.5, 0.2, 0.2, model="m2", priority=50)
    kept, conflicts = merge_detections([a, b], iou_threshold=0.5)
    assert len(kept) == 1
    assert kept[0].conf == pytest.approx(0.9)
    assert conflicts == 1  # different classes


def test_merge_iou_threshold_below_keeps_both() -> None:
    # Box A at (0.3, 0.5), Box B at (0.7, 0.5), both size 0.2 -> no overlap.
    a = _det(0, 0.9, 0.3, 0.5, 0.2, 0.2)
    b = _det(0, 0.9, 0.7, 0.5, 0.2, 0.2)
    kept, _ = merge_detections([a, b], iou_threshold=0.5)
    assert len(kept) == 2


# --------------------------------------------------------------------------- #
# Source-class-id mapping
# --------------------------------------------------------------------------- #


def test_map_to_tp_id_uses_classes_list() -> None:
    from vision_training_collector.auto_annotate import ModelSpec, _map_to_tp_id

    spec = ModelSpec(
        name="v2",
        path=Path("x.pt"),
        priority=100,
        conf_threshold=0.3,
        classes=["Car", "Truck", "Bus"],
    )
    name_to_id = {"Car": 0, "Truck": 1, "Bus": 3}
    assert _map_to_tp_id(0, spec, {}, name_to_id) == 0
    assert _map_to_tp_id(2, spec, {}, name_to_id) == 3
    # Out-of-range class id -> dropped.
    assert _map_to_tp_id(99, spec, {}, name_to_id) is None


def test_map_to_tp_id_uses_class_mapping() -> None:
    from vision_training_collector.auto_annotate import ModelSpec, _map_to_tp_id

    spec = ModelSpec(
        name="coco",
        path=Path("yolov8x.pt"),
        priority=50,
        conf_threshold=0.4,
        class_mapping={"car": "Car", "truck": "Truck"},
    )
    coco_names = {0: "person", 2: "car", 7: "truck"}
    name_to_id = {"Car": 0, "Truck": 1}
    assert _map_to_tp_id(2, spec, coco_names, name_to_id) == 0
    assert _map_to_tp_id(7, spec, coco_names, name_to_id) == 1
    # COCO "person" is not in mapping -> dropped.
    assert _map_to_tp_id(0, spec, coco_names, name_to_id) is None


# --------------------------------------------------------------------------- #
# Full-pipeline smoke test with mocked inference
# --------------------------------------------------------------------------- #


class _MockModel:
    def __init__(self, names: dict[int, str]):
        self.names = names


def _make_img(path: Path, color: tuple[int, int, int]) -> None:
    img = np.full((180, 320, 3), color, dtype=np.uint8)
    cv2.imwrite(str(path), img)


def _patch_models(
    monkeypatch: pytest.MonkeyPatch,
    per_model_dets: dict[str, dict[str, list[tuple]]],
    model_names: dict[str, dict[int, str]],
) -> None:
    """Replace _load_model and _infer with mocks keyed on the model file basename."""

    def _fake_load(model_path: Path):
        key = model_path.name  # e.g. "best.pt" or "yolov8x.pt"
        names = model_names.get(key, {})
        m = _MockModel(names)
        m._mock_key = key  # type: ignore[attr-defined]
        return m

    def _fake_infer(model: Any, image_path: Path, conf_min: float):
        key = getattr(model, "_mock_key", "")
        dets = per_model_dets.get(key, {}).get(image_path.name, [])
        return [d for d in dets if d[1] >= conf_min]

    monkeypatch.setattr(aa, "_load_model", _fake_load)
    monkeypatch.setattr(aa, "_infer", _fake_infer)


def _write_config(tmp_path: Path) -> Path:
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(
        """
iou_threshold: 0.5
truckpilot_classes:
  0: Car
  1: Truck
  3: Bus
  7: TrafficLightRed
models:
  - name: v2
    path: best.pt
    classes: [Car, Truck, Bus, TrafficLightRed]
    priority: 100
    conf_threshold: 0.30
  - name: coco
    path: yolov8x.pt
    class_mapping:
      car: Car
      truck: Truck
      bus: Bus
    priority: 50
    conf_threshold: 0.40
""",
        encoding="utf-8",
    )
    return cfg_path


def test_pipeline_split_aware_merge(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    in_dir = tmp_path / "in"
    out_dir = tmp_path / "out"
    (in_dir / "train").mkdir(parents=True)
    (in_dir / "val").mkdir(parents=True)

    _make_img(in_dir / "train" / "a.jpg", (255, 0, 0))
    _make_img(in_dir / "train" / "b.jpg", (0, 255, 0))
    _make_img(in_dir / "val" / "v.jpg", (0, 0, 255))

    # v2 detections: (src_class_id, conf, cx, cy, w, h)
    # coco detections in COCO id space (matched via names dict).
    per_model_dets = {
        "best.pt": {
            # a.jpg: v2 sees one Car (high conf)
            "a.jpg": [(0, 0.9, 0.5, 0.5, 0.2, 0.2)],
            # b.jpg: v2 sees nothing
            # v.jpg: v2 sees a TrafficLightRed
            "v.jpg": [(3, 0.8, 0.3, 0.3, 0.1, 0.1)],
        },
        "yolov8x.pt": {
            # a.jpg: COCO also sees a "truck" overlapping the Car -> v2 wins, conflict++
            "a.jpg": [(7, 0.95, 0.5, 0.5, 0.2, 0.2)],
            # b.jpg: COCO sees a "bus" not seen by v2 -> kept
            "b.jpg": [(5, 0.85, 0.4, 0.4, 0.2, 0.2)],
            # COCO also sees a "person" (not mapped) -> dropped
            "v.jpg": [(0, 0.99, 0.7, 0.7, 0.2, 0.2)],
        },
    }
    model_names = {
        "best.pt": {},  # classes-list spec; names not consulted
        "yolov8x.pt": {0: "person", 5: "bus", 7: "truck"},
    }
    _patch_models(monkeypatch, per_model_dets, model_names)

    cfg = AutoAnnotateConfig.from_yaml(_write_config(tmp_path))
    report = auto_annotate_directory(in_dir, out_dir, cfg)

    assert report["processed"] == 3
    assert report["splits"] == {"train": 2, "val": 1}
    # merged: a -> 1 (Car, conflict with coco truck), b -> 1 (Bus from coco), v -> 1 (TrafficLightRed)
    assert report["merged_total"] == 3
    assert report["conflicts_total"] == 1
    assert report["detections_per_class"]["Car"] == 1
    assert report["detections_per_class"]["Bus"] == 1
    assert report["detections_per_class"]["TrafficLightRed"] == 1
    assert report["detections_per_class"].get("Truck", 0) == 0

    # Per-model kept counts:
    by_model = {m["name"]: m for m in report["models"]}
    assert by_model["v2"]["detections_kept"] == 2  # Car (a) + TrafficLightRed (v)
    assert by_model["coco"]["detections_kept"] == 1  # Bus (b)

    # Label files exist with one line each.
    assert (out_dir / "labels" / "train" / "a.txt").read_text().strip().split()[0] == "0"
    assert (out_dir / "labels" / "train" / "b.txt").read_text().strip().split()[0] == "3"
    assert (out_dir / "labels" / "val" / "v.txt").read_text().strip().split()[0] == "7"

    # Manifest has per-model columns
    import csv

    with (out_dir / "auto_annotate_manifest.csv").open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    by_file = {r["filename"]: r for r in rows}
    assert by_file["images/train/a.jpg"]["detections_v2"] == "1"
    assert by_file["images/train/a.jpg"]["detections_coco"] == "1"
    assert by_file["images/train/a.jpg"]["conflicts"] == "1"
    assert by_file["images/train/b.jpg"]["detections_v2"] == "0"
    assert by_file["images/train/b.jpg"]["detections_coco"] == "1"


def test_pipeline_rejects_unknown_truckpilot_class(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(
        """
truckpilot_classes:
  0: Car
models:
  - name: v2
    path: best.pt
    classes: [Car, NotAClass]
    priority: 100
    conf_threshold: 0.30
""",
        encoding="utf-8",
    )
    in_dir = tmp_path / "in"
    in_dir.mkdir()
    _make_img(in_dir / "x.jpg", (255, 0, 0))
    _patch_models(monkeypatch, {"best.pt": {}}, {"best.pt": {}})

    cfg = AutoAnnotateConfig.from_yaml(cfg_path)
    with pytest.raises(ValueError, match="unknown TruckPilot class names"):
        auto_annotate_directory(in_dir, tmp_path / "out", cfg)
