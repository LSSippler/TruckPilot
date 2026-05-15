"""Auto-annotate TruckPilot frames with multiple YOLO models and merge results.

Replaces the single-model ETS2LA-pre-labeler. Each model is loaded once,
inference is run per image, detections are normalized to the TruckPilot
class id space, and a greedy box-NMS merges all detections across models.
On overlap (IoU > iou_threshold) the higher-priority model wins; ties are
broken by confidence.

Config (YAML)::

    truckpilot_classes:
      0: Car
      1: Truck
      ...

    iou_threshold: 0.5

    models:
      - name: truckpilot_v2
        path: models/truckpilot-yolov8s-v2/best.pt
        # ordered list: index = model's output class id, value = TruckPilot name
        classes: [Car, Truck, ...]
        priority: 100
        conf_threshold: 0.35

      - name: yolov8x_coco
        path: yolov8x.pt
        # COCO name (model.names value) -> TruckPilot name
        class_mapping:
          car: Car
          truck: Truck
        priority: 50
        conf_threshold: 0.40
"""

from __future__ import annotations

import csv
import json
import logging
import pathlib
import platform
import shutil
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from tqdm import tqdm

_SPLIT_NAMES = ("train", "val", "test")
_FLAT_SPLIT = "flat"

log = logging.getLogger(__name__)

if platform.system() == "Windows":
    pathlib.PosixPath = pathlib.WindowsPath  # type: ignore[misc,assignment]


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #


@dataclass
class ModelSpec:
    name: str
    path: Path
    priority: int
    conf_threshold: float
    # Exactly one of these two is set after normalisation.
    # classes: index = source class id, value = TruckPilot name
    classes: list[str] | None = None
    # class_mapping: source class name -> TruckPilot name
    class_mapping: dict[str, str] | None = None


@dataclass
class AutoAnnotateConfig:
    truckpilot_classes: dict[int, str] = field(default_factory=dict)
    iou_threshold: float = 0.5
    models: list[ModelSpec] = field(default_factory=list)

    @property
    def name_to_id(self) -> dict[str, int]:
        return {v: k for k, v in self.truckpilot_classes.items()}

    @classmethod
    def from_yaml(cls, path: Path) -> "AutoAnnotateConfig":
        import yaml  # type: ignore[import-not-found]

        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        tp_classes = {int(k): str(v) for k, v in (data.get("truckpilot_classes") or {}).items()}
        if not tp_classes:
            raise ValueError("config missing truckpilot_classes")

        models: list[ModelSpec] = []
        for entry in data.get("models") or []:
            if "classes" in entry and "class_mapping" in entry:
                raise ValueError(
                    f"model {entry.get('name')!r}: use either 'classes' or 'class_mapping', not both"
                )
            if "classes" not in entry and "class_mapping" not in entry:
                raise ValueError(
                    f"model {entry.get('name')!r}: must declare 'classes' or 'class_mapping'"
                )
            models.append(
                ModelSpec(
                    name=str(entry["name"]),
                    path=Path(str(entry["path"])),
                    priority=int(entry.get("priority", 0)),
                    conf_threshold=float(entry.get("conf_threshold", 0.25)),
                    classes=[str(c) for c in entry["classes"]] if "classes" in entry else None,
                    class_mapping=(
                        {str(k): str(v) for k, v in entry["class_mapping"].items()}
                        if "class_mapping" in entry
                        else None
                    ),
                )
            )
        if not models:
            raise ValueError("config has no models")

        return cls(
            truckpilot_classes=tp_classes,
            iou_threshold=float(data.get("iou_threshold", 0.5)),
            models=models,
        )


# --------------------------------------------------------------------------- #
# Detection record
# --------------------------------------------------------------------------- #


@dataclass
class Detection:
    """Normalized detection in TruckPilot class id space.

    Coordinates are YOLO-normalized (cx, cy, w, h in [0, 1]).
    """

    tp_class_id: int
    conf: float
    cx: float
    cy: float
    w: float
    h: float
    model_name: str
    priority: int

    def as_xyxy(self) -> tuple[float, float, float, float]:
        x1 = self.cx - self.w / 2.0
        y1 = self.cy - self.h / 2.0
        x2 = self.cx + self.w / 2.0
        y2 = self.cy + self.h / 2.0
        return (x1, y1, x2, y2)


# --------------------------------------------------------------------------- #
# Model loader / inference adapter
# --------------------------------------------------------------------------- #


def _load_model(model_path: Path) -> Any:
    """Load via ultralytics.YOLO. Path may be a local .pt or a known short name
    (e.g. "yolov8x.pt") which ultralytics will auto-download.
    """
    from ultralytics import YOLO  # type: ignore[import-not-found]

    # Allow short names that ultralytics resolves itself (auto-download).
    if not model_path.is_absolute() and not model_path.exists():
        log.info("loading model via ultralytics short-name (auto-download if needed): %s", model_path)
        return YOLO(str(model_path))
    if not model_path.exists():
        raise FileNotFoundError(f"model not found: {model_path}")
    log.info("loading ultralytics.YOLO: %s", model_path)
    return YOLO(str(model_path))


def _model_names(model: Any) -> dict[int, str]:
    """Return {class_id: class_name} as reported by the loaded model."""
    names = getattr(model, "names", None)
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}
    if isinstance(names, (list, tuple)):
        return {i: str(n) for i, n in enumerate(names)}
    return {}


def _infer(model: Any, image_path: Path, conf_min: float) -> list[tuple[int, float, float, float, float, float]]:
    """Run ultralytics inference. Returns (src_class_id, conf, cx, cy, w, h) normalized."""
    results = model.predict(source=str(image_path), conf=conf_min, verbose=False)
    if not results:
        return []
    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        return []
    h, w = r.orig_shape
    xyxy = r.boxes.xyxy.cpu().numpy()
    confs = r.boxes.conf.cpu().numpy()
    clss = r.boxes.cls.cpu().numpy().astype(int)
    out: list[tuple[int, float, float, float, float, float]] = []
    for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
        cx = ((x1 + x2) / 2.0) / w
        cy = ((y1 + y2) / 2.0) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h
        out.append((int(k), float(c), float(cx), float(cy), float(bw), float(bh)))
    return out


# --------------------------------------------------------------------------- #
# Per-model normalisation
# --------------------------------------------------------------------------- #


def _map_to_tp_id(
    src_class_id: int,
    spec: ModelSpec,
    model_names: dict[int, str],
    tp_name_to_id: dict[str, int],
) -> int | None:
    """Translate a source class id to a TruckPilot class id, or None to drop."""
    if spec.classes is not None:
        if 0 <= src_class_id < len(spec.classes):
            tp_name = spec.classes[src_class_id]
        else:
            return None
    else:
        assert spec.class_mapping is not None
        src_name = model_names.get(src_class_id)
        if src_name is None:
            return None
        tp_name = spec.class_mapping.get(src_name)
        if tp_name is None:
            return None
    return tp_name_to_id.get(tp_name)


# --------------------------------------------------------------------------- #
# NMS merge
# --------------------------------------------------------------------------- #


def _iou(a: Detection, b: Detection) -> float:
    ax1, ay1, ax2, ay2 = a.as_xyxy()
    bx1, by1, bx2, by2 = b.as_xyxy()
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def merge_detections(
    detections: list[Detection], iou_threshold: float
) -> tuple[list[Detection], int]:
    """Greedy NMS across all detections, ignoring class.

    Sort by (priority desc, conf desc). Walk in order; keep a detection if it
    has no IoU > threshold with any already-kept detection. Returns the kept
    list plus the count of suppressed boxes that *would* have produced a
    different TruckPilot class id than the keeper (the "conflict" count).
    """
    if not detections:
        return [], 0
    ordered = sorted(detections, key=lambda d: (-d.priority, -d.conf))
    kept: list[Detection] = []
    conflicts = 0
    for det in ordered:
        clash_keeper: Detection | None = None
        for k in kept:
            if _iou(det, k) > iou_threshold:
                clash_keeper = k
                break
        if clash_keeper is None:
            kept.append(det)
        elif clash_keeper.tp_class_id != det.tp_class_id:
            conflicts += 1
    return kept, conflicts


# --------------------------------------------------------------------------- #
# IO helpers
# --------------------------------------------------------------------------- #


def _format_label(class_id: int, cx: float, cy: float, w: float, h: float) -> str:
    return f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"


def _copy_or_link(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        return
    try:
        dst.symlink_to(src.resolve())
    except (OSError, NotImplementedError):
        shutil.copy2(src, dst)


def _detect_splits(input_dir: Path, exts: set[str]) -> dict[str, list[Path]]:
    """Mirror pre_label's split detection (train/val/test or flat)."""
    found: dict[str, list[Path]] = {}
    if any((input_dir / s).is_dir() for s in _SPLIT_NAMES):
        for split in _SPLIT_NAMES:
            sub = input_dir / split
            if not sub.is_dir():
                continue
            imgs = sorted(p for p in sub.rglob("*") if p.suffix.lower() in exts)
            if imgs:
                found[split] = imgs
        return found
    flat = sorted(p for p in input_dir.rglob("*") if p.suffix.lower() in exts)
    if flat:
        found[_FLAT_SPLIT] = flat
    return found


# --------------------------------------------------------------------------- #
# Pipeline
# --------------------------------------------------------------------------- #


def auto_annotate_directory(
    input_dir: Path,
    output_dir: Path,
    config: AutoAnnotateConfig,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Run multi-model auto-annotation over input_dir.

    Output layout::

        <output>/images/<split>/<stem>.<ext>           (copy or symlink of input)
        <output>/labels/<split>/<stem>.txt             (merged YOLO labels)
        <output>/auto_annotate_manifest.csv            (per-image counts)
        <output>/auto_annotate_report.json             (aggregate stats)
        <output>/auto_annotate_config.snapshot.yaml    (mirror for reproducibility)
    """
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    if not input_dir.exists():
        raise FileNotFoundError(f"input dir not found: {input_dir}")

    if not dry_run:
        output_dir.mkdir(parents=True, exist_ok=True)
        try:
            probe = output_dir / ".write_probe"
            probe.write_text("ok", encoding="utf-8")
            probe.unlink()
        except OSError as exc:
            raise RuntimeError(f"output_dir not writable: {output_dir} ({exc})") from exc

    # Load all models up-front; record names for COCO-style class_mapping.
    loaded: list[tuple[ModelSpec, Any, dict[int, str]]] = []
    for spec in config.models:
        model = _load_model(spec.path)
        loaded.append((spec, model, _model_names(model)))

    tp_name_to_id = config.name_to_id
    # Sanity check spec classes/class_mapping resolve to known TruckPilot names.
    for spec, _model, _names in loaded:
        unknown: list[str] = []
        if spec.classes is not None:
            unknown += [n for n in spec.classes if n not in tp_name_to_id]
        if spec.class_mapping is not None:
            unknown += [n for n in spec.class_mapping.values() if n not in tp_name_to_id]
        if unknown:
            raise ValueError(
                f"model {spec.name!r} references unknown TruckPilot class names: "
                f"{sorted(set(unknown))}"
            )

    exts = {".jpg", ".jpeg", ".png"}
    splits_to_images = _detect_splits(input_dir, exts)
    total_images = sum(len(v) for v in splits_to_images.values())
    log.info(
        "auto-annotating %d images across %d split(s) %s with %d model(s)",
        total_images,
        len(splits_to_images),
        list(splits_to_images.keys()),
        len(loaded),
    )

    report: dict[str, Any] = {
        "input_images": total_images,
        "processed": 0,
        "skipped": 0,
        "errors": [],
        "splits": {s: len(v) for s, v in splits_to_images.items()},
        "iou_threshold": config.iou_threshold,
        "models": [
            {
                "name": spec.name,
                "path": str(spec.path),
                "priority": spec.priority,
                "conf_threshold": spec.conf_threshold,
                "detections_kept": 0,
                "detections_total": 0,
            }
            for spec, _m, _n in loaded
        ],
        "detections_per_class": {name: 0 for name in config.truckpilot_classes.values()},
        "merged_total": 0,
        "conflicts_total": 0,
        "images_without_detections": 0,
        "inference_time_seconds": 0.0,
        "run_timestamp": datetime.now(timezone.utc).isoformat(),
        "dry_run": dry_run,
    }

    manifest_rows: list[dict[str, Any]] = []
    t_start = time.time()

    # Column names: dynamic per-model count columns.
    per_model_cols = [f"detections_{spec.name}" for spec, _m, _n in loaded]

    for split, images in splits_to_images.items():
        if not dry_run:
            (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

        for img_path in tqdm(images, desc=f"Auto-annotate [{split}]", unit="img"):
            per_model_counts: dict[str, int] = {spec.name: 0 for spec, _m, _n in loaded}
            all_dets: list[Detection] = []
            failed = False

            for idx, (spec, model, names) in enumerate(loaded):
                try:
                    raw = _infer(model, img_path, conf_min=spec.conf_threshold)
                except Exception as exc:  # noqa: BLE001
                    log.warning("inference failed (%s) on %s: %s", spec.name, img_path.name, exc)
                    report["errors"].append(
                        {"file": str(img_path), "model": spec.name, "error": str(exc)}
                    )
                    failed = True
                    break

                report["models"][idx]["detections_total"] += len(raw)
                for src_id, conf, cx, cy, w, h in raw:
                    tp_id = _map_to_tp_id(src_id, spec, names, tp_name_to_id)
                    if tp_id is None:
                        continue
                    det = Detection(
                        tp_class_id=tp_id,
                        conf=conf,
                        cx=cx,
                        cy=cy,
                        w=w,
                        h=h,
                        model_name=spec.name,
                        priority=spec.priority,
                    )
                    all_dets.append(det)
                    per_model_counts[spec.name] += 1

            if failed:
                report["skipped"] += 1
                continue

            kept, conflicts = merge_detections(all_dets, config.iou_threshold)
            kept_lines: list[str] = []
            for det in kept:
                kept_lines.append(_format_label(det.tp_class_id, det.cx, det.cy, det.w, det.h))
                name = config.truckpilot_classes.get(det.tp_class_id, f"class_{det.tp_class_id}")
                report["detections_per_class"][name] = (
                    report["detections_per_class"].get(name, 0) + 1
                )
                # Per-model kept attribution.
                for m in report["models"]:
                    if m["name"] == det.model_name:
                        m["detections_kept"] += 1
                        break

            report["processed"] += 1
            report["merged_total"] += len(kept_lines)
            report["conflicts_total"] += conflicts
            if not kept_lines:
                report["images_without_detections"] += 1

            rel = (
                f"images/{split}/{img_path.name}"
                if split != _FLAT_SPLIT
                else f"images/{img_path.name}"
            )
            row: dict[str, Any] = {
                "filename": rel,
                "split": split,
                "merged_total": len(kept_lines),
                "conflicts": conflicts,
            }
            for name, count in per_model_counts.items():
                row[f"detections_{name}"] = count
            manifest_rows.append(row)

            if dry_run:
                continue

            img_dst_dir = (
                (output_dir / "images" / split) if split != _FLAT_SPLIT else (output_dir / "images")
            )
            lbl_dst_dir = (
                (output_dir / "labels" / split) if split != _FLAT_SPLIT else (output_dir / "labels")
            )
            img_dst_dir.mkdir(parents=True, exist_ok=True)
            lbl_dst_dir.mkdir(parents=True, exist_ok=True)
            _copy_or_link(img_path, img_dst_dir / img_path.name)
            stem = img_path.stem
            label_path = lbl_dst_dir / f"{stem}.txt"
            if kept_lines:
                label_path.write_text("\n".join(kept_lines) + "\n", encoding="utf-8")
            else:
                # Empty label file: explicit "no objects" annotation for YOLO trainers.
                # We deliberately *do* write an empty file here (vs pre_label which
                # left it missing) so reviewers can tell "auto-annotated, nothing
                # found" apart from "never processed".
                label_path.write_text("", encoding="utf-8")

    report["inference_time_seconds"] = round(time.time() - t_start, 2)

    if not dry_run:
        manifest_path = output_dir / "auto_annotate_manifest.csv"
        fieldnames = ["filename", "split", "merged_total", "conflicts", *per_model_cols]
        with manifest_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(manifest_rows)
        (output_dir / "auto_annotate_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    log.info(
        "auto-annotate done: %d processed, %d merged dets, %d conflicts, %d errors (%.1fs)",
        report["processed"],
        report["merged_total"],
        report["conflicts_total"],
        len(report["errors"]),
        report["inference_time_seconds"],
    )
    return report
