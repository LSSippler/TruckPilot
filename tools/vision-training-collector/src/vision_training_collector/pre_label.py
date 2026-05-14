"""Pre-label TruckPilot frames with the ETS2LA YOLOv5s model.

Loads ETS2LA's YOLOv5s checkpoint, runs inference over a directory of frames,
maps ETS2LA class IDs onto TruckPilot's 15-class Phase-1 schema, and sorts the
resulting YOLO label files into auto / review / (manual placeholder) folders
based on detection confidence.
"""

from __future__ import annotations

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


def pre_label_directory(
    input_dir: Path,
    output_dir: Path,
    config: ClassMapping,
    model_path: Path,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Run pre-labeling over input_dir, write labels + report into output_dir."""
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    if not input_dir.exists():
        raise FileNotFoundError(f"input dir not found: {input_dir}")

    images_root = output_dir / "images"
    labels_auto = output_dir / "labels" / "auto"
    labels_review = output_dir / "labels" / "review"
    labels_manual = output_dir / "labels" / "manual"
    if not dry_run:
        for d in (images_root, labels_auto, labels_review, labels_manual):
            d.mkdir(parents=True, exist_ok=True)
        # writability probe
        try:
            probe = output_dir / ".write_probe"
            probe.write_text("ok", encoding="utf-8")
            probe.unlink()
        except OSError as exc:
            raise RuntimeError(f"output_dir not writable: {output_dir} ({exc})") from exc

    model = _load_model(model_path)

    exts = {".jpg", ".jpeg", ".png"}
    images = sorted([p for p in input_dir.rglob("*") if p.suffix.lower() in exts])
    log.info("pre-labeling %d images from %s", len(images), input_dir)

    report: dict[str, Any] = {
        "input_images": len(images),
        "processed": 0,
        "skipped": 0,
        "errors": [],
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

    t_start = time.time()

    for img_path in tqdm(images, desc="Pre-labeling", unit="img"):
        try:
            dets = _infer(model, img_path, conf_min=config.review_min)
        except Exception as exc:  # noqa: BLE001
            log.warning("inference failed for %s: %s", img_path.name, exc)
            report["errors"].append({"file": str(img_path), "error": str(exc)})
            report["skipped"] += 1
            continue

        auto_lines: list[str] = []
        review_lines: list[str] = []

        for ets2la_id, conf, cx, cy, w, h in dets:
            if conf < config.review_min:
                report["detections_per_tier"]["dropped_low_conf"] += 1
                continue
            tp_id = config.map_class(ets2la_id)
            if tp_id is None:
                report["detections_per_tier"]["dropped_unmapped"] += 1
                continue
            line = _format_label(tp_id, cx, cy, w, h)
            if conf >= config.auto_accept:
                auto_lines.append(line)
                report["detections_per_tier"]["auto_accept"] += 1
            else:
                review_lines.append(line)
                report["detections_per_tier"]["review"] += 1
            name = config.truckpilot_classes.get(tp_id, f"class_{tp_id}")
            report["detections_per_class"][name] = report["detections_per_class"].get(name, 0) + 1

        report["processed"] += 1
        if not auto_lines and not review_lines:
            report["images_without_detections"] += 1

        if dry_run:
            continue

        # copy/link image once
        _copy_or_link(img_path, images_root / img_path.name)

        stem = img_path.stem
        if auto_lines:
            (labels_auto / f"{stem}.txt").write_text("\n".join(auto_lines) + "\n", encoding="utf-8")
        if review_lines:
            (labels_review / f"{stem}.txt").write_text("\n".join(review_lines) + "\n", encoding="utf-8")
        # touch an empty manual file so reviewers see the slot (only when nothing else exists)
        if not auto_lines and not review_lines:
            (labels_manual / f"{stem}.txt").touch()

    report["inference_time_seconds"] = round(time.time() - t_start, 2)

    if not dry_run:
        # mirror the mapping yaml next to the report for reproducibility
        try:
            src_yaml = Path(__file__).resolve().parent.parent.parent / "class_mapping.yaml"
            if src_yaml.exists():
                shutil.copy2(src_yaml, output_dir / "class_mapping.yaml")
        except OSError:
            pass
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
