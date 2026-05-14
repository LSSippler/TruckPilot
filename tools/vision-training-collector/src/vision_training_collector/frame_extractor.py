"""Video -> frames extraction with skip heuristics."""

from __future__ import annotations

import logging
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm

from .config import Config
from .state import State

log = logging.getLogger(__name__)


def _is_black(frame: np.ndarray, threshold: float) -> bool:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    mean = float(gray.mean()) / 255.0
    return mean < threshold


def _is_menu(frame: np.ndarray, max_unique_colors: int) -> bool:
    # downscale to make unique-color count tractable
    small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
    flat = small.reshape(-1, small.shape[-1]) if small.ndim == 3 else small.reshape(-1, 1)
    # quantize to 5-bit per channel
    quant = (flat // 8).astype(np.uint8)
    unique = np.unique(quant, axis=0)
    return len(unique) < max_unique_colors


def _normalize(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    h, w = frame.shape[:2]
    if (w, h) == (width, height):
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def extract_video(
    video_path: Path,
    out_dir: Path,
    cfg: Config,
    state: State | None = None,
) -> dict[str, int]:
    """Extract frames from a single video. Returns stats."""
    out_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        log.error("cannot open video: %s", video_path)
        return {"read": 0, "saved": 0, "skipped_black": 0, "skipped_menu": 0}

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps * cfg.extract.frame_interval_seconds)))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    log.info("extracting %s (fps=%.1f step=%d total=%d)", video_path.name, fps, step, total)

    stats = {"read": 0, "saved": 0, "skipped_black": 0, "skipped_menu": 0}
    idx = 0
    saved_idx = 0

    pbar = tqdm(
        total=total if total > 0 else None,
        desc=f"Extracting {video_path.stem}",
        unit="frame",
        unit_scale=False,
        smoothing=0.1,
    )

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        stats["read"] += 1
        pbar.update(1)
        pbar.set_postfix(saved=stats["saved"], black=stats["skipped_black"], menu=stats["skipped_menu"])
        if idx % step != 0:
            idx += 1
            continue
        idx += 1
        if _is_black(frame, cfg.extract.skip_black_threshold):
            stats["skipped_black"] += 1
            continue
        if _is_menu(frame, cfg.extract.skip_menu_color_count):
            stats["skipped_menu"] += 1
            continue
        norm = _normalize(frame, cfg.extract.target_width, cfg.extract.target_height)
        out_path = out_dir / f"frame_{saved_idx:06d}.jpg"
        cv2.imwrite(str(out_path), norm, [cv2.IMWRITE_JPEG_QUALITY, 92])
        saved_idx += 1
        stats["saved"] += 1
    pbar.close()
    cap.release()

    if state is not None:
        state.add_to_set("extracted_videos", video_path.stem)
    log.info("done %s: %s", video_path.name, stats)
    return stats


def extract_all(raw_dir: Path, frames_dir: Path, cfg: Config, state_path: Path) -> dict[str, int]:
    state = State(state_path)
    totals = {"videos": 0, "saved": 0, "skipped_black": 0, "skipped_menu": 0}
    for video in sorted(raw_dir.glob("*.mp4")) + sorted(raw_dir.glob("*.mkv")) + sorted(raw_dir.glob("*.webm")):
        if state.has("extracted_videos", video.stem):
            log.info("skip already-extracted: %s", video.name)
            continue
        out = frames_dir / video.stem
        s = extract_video(video, out, cfg, state)
        totals["videos"] += 1
        totals["saved"] += s["saved"]
        totals["skipped_black"] += s["skipped_black"]
        totals["skipped_menu"] += s["skipped_menu"]
    log.info("extract-all totals: %s", totals)
    return totals
