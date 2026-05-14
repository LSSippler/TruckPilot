"""Pre-label TruckPilot frames with the ETS2LA YOLOv5s model.

Loads ETS2LA's YOLOv5s checkpoint, runs inference over a directory of frames,
maps ETS2LA class IDs onto TruckPilot's 15-class Phase-1 schema, and sorts the
resulting YOLO label files into auto / review / (manual placeholder) folders
based on detection confidence.
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

# Standard YOLO train/val/test split names. If `input_dir` contains any of
# these as direct subfolders, the pre-label pipeline mirrors that structure
# into the output (images/<split>/, labels/<split>/). Otherwise it falls back
# to flat output (images/<file>, labels/<file>) and the manifest's "split"
# column is "flat".
_SPLIT_NAMES = ("train", "val", "test")
_FLAT_SPLIT = "flat"

log = logging.getLogger(__name__)

# Windows fix for checkpoints pickled on POSIX (ETS2LA ships .pt with PosixPath).
if platform.system() == "Windows":
    pathlib.PosixPath = pathlib.WindowsPath  # type: ignore[misc,assignment]


# --------------------------------------------------------------------------- #
# Mapping
# --------------------------------------------------------------------------- #


@dataclass
class ClassMapping:
    ets2la_to_truckpilot: dict[int, int] = field(default_factory=dict)
    ets2la_suppress: set[int] = field(default_factory=set)
    truckpilot_classes: dict[int, str] = field(default_factory=dict)
    manual_only_classes: set[int] = field(default_factory=set)
    auto_accept: float = 0.85
    review_min: float = 0.30

    @classmethod
    def from_yaml(cls, path: Path) -> "ClassMapping":
        try:
            import yaml  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "PyYAML is required to load class_mapping.yaml (pip install pyyaml)."
            ) from exc
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        tiers = data.get("confidence_tiers") or {}
        return cls(
            ets2la_to_truckpilot={int(k): int(v) for k, v in (data.get("ets2la_to_truckpilot") or {}).items()},
            ets2la_suppress=set(int(x) for x in (data.get("ets2la_suppress") or [])),
            truckpilot_classes={int(k): str(v) for k, v in (data.get("truckpilot_classes") or {}).items()},
            manual_only_classes=set(int(x) for x in (data.get("manual_only_classes") or [])),
            auto_accept=float(tiers.get("auto_accept", 0.85)),
            review_min=float(tiers.get("review_min", 0.30)),
        )

    def map_class(self, ets2la_id: int) -> int | None:
        """Return TruckPilot class id, or None if the detection should be dropped."""
        if ets2la_id in self.ets2la_suppress:
            return None
        return self.ets2la_to_truckpilot.get(ets2la_id)


# --------------------------------------------------------------------------- #
# Model loader
# --------------------------------------------------------------------------- #


def _load_model(model_path: Path) -> Any:
    """Load ETS2LA YOLOv5s. Tries ultralytics YOLO first, falls back to torch.hub."""
    if not model_path.exists():
        raise FileNotFoundError(f"model not found: {model_path}")

    # Preferred: ultralytics package (newer, no internet fetch).
    try:
        from ultralytics import YOLO  # type: ignore[import-not-found]
        log.info("loading model via ultralytics.YOLO: %s", model_path)
        return YOLO(str(model_path))
    except Exception as exc:  # noqa: BLE001
        log.warning("ultralytics load failed (%s); trying torch.hub", exc)

    try:
        import torch  # type: ignore[import-not-found]
        log.info("loading model via torch.hub (ultralytics/yolov5)")
        model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=str(model_path),
            force_reload=False,
            trust_repo=True,
        )
        return model
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"could not load model {model_path}: {exc}") from exc


def _is_ultralytics(model: Any) -> bool:
    return model.__class__.__module__.startswith("ultralytics")


# --------------------------------------------------------------------------- #
# Inference adapter — returns list[(ets2la_class_id, conf, cx, cy, w, h)] normalized 0-1
# --------------------------------------------------------------------------- #


def _infer(model: Any, image_path: Path, conf_min: float) -> list[tuple[int, float, float, float, float, float]]:
    if _is_ultralytics(model):
        results = model.predict(source=str(image_path), conf=conf_min, verbose=False)
        if not results:
            return []
        r = results[0]
        h, w = r.orig_shape  # (h, w)
        out: list[tuple[int, float, float, float, float, float]] = []
        if r.boxes is None or len(r.boxes) == 0:
            return out
        xyxy = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        clss = r.boxes.cls.cpu().numpy().astype(int)
        for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
            cx = ((x1 + x2) / 2.0) / w
            cy = ((y1 + y2) / 2.0) / h
            bw = (x2 - x1) / w
            bh = (y2 - y1) / h
            out.append((int(k), float(c), float(cx), float(cy), float(bw), float(bh)))
        return out

    # torch.hub YOLOv5 path
    results = model(str(image_path))
    df = results.pandas().xywhn[0]  # normalized cx,cy,w,h
    out = []
    for _, row in df.iterrows():
        c = float(row["confidence"])
        if c < conf_min:
            continue
        out.append(
            (
                int(row["class"]),
                c,
                float(row["xcenter"]),
                float(row["ycenter"]),
                float(row["width"]),
                float(row["height"]),
            )
        )
    return out


# --------------------------------------------------------------------------- #
# Pipeline
# --------------------------------------------------------------------------- #


def _format_label(class_id: int, cx: float, cy: float, w: float, h: float) -> str:
    return f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"


def _copy_or_link(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        return
    try:
        # symlink keeps disk usage low when supported; fall back to copy.
        dst.symlink_to(src.resolve())
    except (OSError, NotImplementedError):
        shutil.copy2(src, dst)


def _detect_splits(input_dir: Path, exts: set[str]) -> dict[str, list[Path]]:
    """Return {split_name: [image_paths]} keyed by detected split subfolders.

    If `input_dir` contains any of `_SPLIT_NAMES` as a direct subdirectory, only
    images inside those are picked up and grouped per split. Otherwise the
    entire tree is treated as a single flat group keyed by `_FLAT_SPLIT`.
    """
    found: dict[str, list[Path]] = {}
    has_split_layout = any((input_dir / s).is_dir() for s in _SPLIT_NAMES)
    if has_split_layout:
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


def pre_label_directory(
    input_dir: Path,
    output_dir: Path,
    config: ClassMapping,
    model_path: Path,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Run pre-labeling over input_dir; write split-aware labels + manifest.

    Output layout:
        <output>/images/<split>/<stem>.<ext>     (copy or symlink of input)
        <output>/labels/<split>/<stem>.txt       (YOLO, all kept detections)
        <output>/pre_label_manifest.csv          (per-image tier + counts)
        <output>/pre_label_report.json           (aggregate stats)
        <output>/class_mapping.yaml              (mirror for reproducibility)

    Flat input falls back to <output>/images/<file> and <output>/labels/<file>
    with split = "flat" in the manifest.
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

    model = _load_model(model_path)

    exts = {".jpg", ".jpeg", ".png"}
    splits_to_images = _detect_splits(input_dir, exts)
    total_images = sum(len(v) for v in splits_to_images.values())
    log.info(
        "pre-labeling %d images across %d split(s) %s from %s",
        total_images,
        len(splits_to_images),
        list(splits_to_images.keys()),
        input_dir,
    )

    report: dict[str, Any] = {
        "input_images": total_images,
        "processed": 0,
        "skipped": 0,
        "errors": [],
        "splits": {s: len(v) for s, v in splits_to_images.items()},
        "detections_per_tier": {
            "auto_accept": 0,
            "review": 0,
            "dropped_low_conf": 0,
            "dropped_unmapped": 0,
        },
        "detections_per_class": {name: 0 for name in config.truckpilot_classes.values()},
        "images_without_detections": 0,
        "model_path": str(model_path),
        "inference_time_seconds": 0.0,
        "run_timestamp": datetime.now(timezone.utc).isoformat(),
        "dry_run": dry_run,
    }

    manifest_rows: list[dict[str, Any]] = []
    t_start = time.time()

    for split, images in splits_to_images.items():
        if not dry_run:
            (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

        for img_path in tqdm(images, desc=f"Pre-labeling [{split}]", unit="img"):
            try:
                dets = _infer(model, img_path, conf_min=config.review_min)
            except Exception as exc:  # noqa: BLE001
                log.warning("inference failed for %s: %s", img_path.name, exc)
                report["errors"].append({"file": str(img_path), "error": str(exc)})
                report["skipped"] += 1
                continue

            kept_lines: list[str] = []
            max_conf = 0.0
            any_auto = False
            any_review = False

            for ets2la_id, conf, cx, cy, w, h in dets:
                if conf < config.review_min:
                    report["detections_per_tier"]["dropped_low_conf"] += 1
                    continue
                tp_id = config.map_class(ets2la_id)
                if tp_id is None:
                    report["detections_per_tier"]["dropped_unmapped"] += 1
                    continue
                kept_lines.append(_format_label(tp_id, cx, cy, w, h))
                if conf >= config.auto_accept:
                    any_auto = True
                    report["detections_per_tier"]["auto_accept"] += 1
                else:
                    any_review = True
                    report["detections_per_tier"]["review"] += 1
                if conf > max_conf:
                    max_conf = conf
                name = config.truckpilot_classes.get(tp_id, f"class_{tp_id}")
                report["detections_per_class"][name] = (
                    report["detections_per_class"].get(name, 0) + 1
                )

            report["processed"] += 1
            if not kept_lines:
                report["images_without_detections"] += 1
                tier = "none"
            else:
                # An image is "auto" only if every kept detection is above the
                # auto-accept threshold; any review-tier detection demotes it.
                tier = "auto" if any_auto and not any_review else "review"

            # filename column uses YOLO-canonical forward slashes for portability.
            rel = f"images/{split}/{img_path.name}" if split != _FLAT_SPLIT else f"images/{img_path.name}"
            manifest_rows.append(
                {
                    "filename": rel,
                    "split": split,
                    "tier": tier,
                    "confidence_max": round(max_conf, 4),
                    "num_detections": len(kept_lines),
                }
            )

            if dry_run:
                continue

            img_dst_dir = (output_dir / "images" / split) if split != _FLAT_SPLIT else (output_dir / "images")
            lbl_dst_dir = (output_dir / "labels" / split) if split != _FLAT_SPLIT else (output_dir / "labels")
            img_dst_dir.mkdir(parents=True, exist_ok=True)
            lbl_dst_dir.mkdir(parents=True, exist_ok=True)
            _copy_or_link(img_path, img_dst_dir / img_path.name)
            stem = img_path.stem
            if kept_lines:
                (lbl_dst_dir / f"{stem}.txt").write_text(
                    "\n".join(kept_lines) + "\n", encoding="utf-8"
                )
            # Images with no detections deliberately get no label file. YOLO
            # treats a missing label as an empty annotation set; reviewers can
            # filter via the manifest's tier=="none" rows.

    report["inference_time_seconds"] = round(time.time() - t_start, 2)

    if not dry_run:
        try:
            src_yaml = Path(__file__).resolve().parent.parent.parent / "class_mapping.yaml"
            if src_yaml.exists():
                shutil.copy2(src_yaml, output_dir / "class_mapping.yaml")
        except OSError:
            pass
        manifest_path = output_dir / "pre_label_manifest.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(
                fh,
                fieldnames=["filename", "split", "tier", "confidence_max", "num_detections"],
            )
            writer.writeheader()
            writer.writerows(manifest_rows)
        (output_dir / "pre_label_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    log.info(
        "pre-label done: %d processed, %d auto, %d review, %d errors (%.1fs)",
        report["processed"],
        report["detections_per_tier"]["auto_accept"],
        report["detections_per_tier"]["review"],
        len(report["errors"]),
        report["inference_time_seconds"],
    )
    return report
