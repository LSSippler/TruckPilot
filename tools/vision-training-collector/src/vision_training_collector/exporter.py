"""Train/Val/Test split + manifest export."""

from __future__ import annotations

import csv
import logging
import random
import shutil
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from .config import Config

log = logging.getLogger(__name__)


def _scene_hint(path: Path, cfg: Config) -> str:
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return "unknown"
    edges = cv2.Canny(img, 80, 180)
    edge_density = float(edges.mean()) / 255.0

    # detect horizontal vs vertical edge bias via Sobel
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
    mag_x = float(np.abs(gx).sum())
    mag_y = float(np.abs(gy).sum())
    horizontal_ratio = mag_x / (mag_x + mag_y + 1e-6)

    if edge_density >= cfg.scene_hint.city_edge_density:
        return "city"
    if horizontal_ratio >= cfg.scene_hint.highway_horizontal_ratio:
        return "highway"
    return "rural"


def _timestamp_from_name(name: str) -> str:
    # frame_000123.jpg -> 000123
    stem = Path(name).stem
    if stem.startswith("frame_"):
        return stem.split("_", 1)[1]
    if stem.startswith("live_"):
        return stem.split("_", 1)[1]
    return stem


def export(src_dir: Path, final_dir: Path, cfg: Config) -> dict[str, int]:
    """Split images from src_dir into final_dir/images/{train,val,test}/ and write manifest.csv."""
    exts = {".jpg", ".jpeg", ".png"}
    files = sorted([p for p in src_dir.rglob("*") if p.suffix.lower() in exts])
    if not files:
        log.warning("no images found in %s", src_dir)
        return {"total": 0, "train": 0, "val": 0, "test": 0}

    ratios = (cfg.export.train_ratio, cfg.export.val_ratio, cfg.export.test_ratio)
    if abs(sum(ratios) - 1.0) > 1e-3:
        raise ValueError(f"train/val/test ratios must sum to 1.0, got {sum(ratios)}")

    rng = random.Random(cfg.export.seed)
    shuffled = files[:]
    rng.shuffle(shuffled)
    n = len(shuffled)
    n_train = int(n * cfg.export.train_ratio)
    n_val = int(n * cfg.export.val_ratio)
    splits = {
        "train": shuffled[:n_train],
        "val": shuffled[n_train : n_train + n_val],
        "test": shuffled[n_train + n_val :],
    }

    images_root = final_dir / "images"
    for split in splits:
        (images_root / split).mkdir(parents=True, exist_ok=True)

    manifest_path = final_dir / "manifest.csv"
    final_dir.mkdir(parents=True, exist_ok=True)

    rows = 0
    with manifest_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["filename", "split", "source_video", "timestamp", "scene_hint", "exported_at"])
        now_iso = datetime.utcnow().isoformat()
        for split, items in splits.items():
            for src in items:
                source_video = src.parent.name
                ts = _timestamp_from_name(src.name)
                scene = _scene_hint(src, cfg)
                dst_name = f"{source_video}__{src.name}"
                dst = images_root / split / dst_name
                shutil.copy2(src, dst)
                writer.writerow([f"images/{split}/{dst_name}", split, source_video, ts, scene, now_iso])
                rows += 1

    # YOLO data.yaml stub for later labeling
    yaml_path = final_dir / "data.yaml"
    yaml_path.write_text(
        "# TruckPilot Phase 6.5d - YOLO dataset stub (labels TBD via Roboflow)\n"
        f"path: {final_dir.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "nc: 15\n"
        "names:\n"
        "  - Car\n  - Truck\n  - TruckTrailer\n  - Bus\n  - BrakeLightOn\n"
        "  - TurnSignalLeft\n  - TurnSignalRight\n  - TrafficLightRed\n"
        "  - TrafficLightYellow\n  - TrafficLightGreen\n  - StopSign\n"
        "  - SpeedLimitSign\n  - LaneSolid\n  - LaneDashed\n  - RoadEdge\n",
        encoding="utf-8",
    )

    stats = {"total": rows, "train": len(splits["train"]), "val": len(splits["val"]), "test": len(splits["test"])}
    log.info("export stats: %s", stats)
    return stats
