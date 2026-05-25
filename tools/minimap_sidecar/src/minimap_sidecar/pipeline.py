"""VMM-2 OpenCV route-line detection pipeline.

Steps (per frame):
  1. Crop ROI from full-frame BGR image.
  2. BGR → HSV → inRange mask (handles red hue wrap-around).
  3. Morphological open (denoise) + close (fill line gaps).
  4. Morphological skeleton via iterative erosion+hit-or-miss.
  5. Extract skeleton pixel coords, sort into a polyline via nearest-neighbour
     traversal starting from the top-most point (closest to truck).
  6. Subsample to ≤MAX_POINTS evenly spaced along the polyline.
  7. Compute confidence from pixel density and line continuity.

Returns a DetectionResult with point list (ROI-local pixel coords) and confidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import cv2
import numpy as np

from . import MAX_POINTS


@dataclass
class DetectionResult:
    points: list[tuple[float, float]]  # pixel coords within ROI
    confidence: float                  # [0.0, 1.0]
    mask_pixel_count: int = 0
    skeleton_pixel_count: int = 0


# ── HSV masking ───────────────────────────────────────────────────────────────

def _hsv_mask(
    roi_bgr: np.ndarray,
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
) -> np.ndarray:
    """Binary mask for pixels in HSV range. Handles red hue wrap-around."""
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    lo = np.array(lower, dtype=np.uint8)
    hi = np.array(upper, dtype=np.uint8)

    if lo[0] <= hi[0]:
        mask = cv2.inRange(hsv, lo, hi)
    else:
        # red wraps: H_lower > H_upper → split at 0 and 180
        mask_a = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], dtype=np.uint8))
        mask_b = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], dtype=np.uint8), hi)
        mask = cv2.bitwise_or(mask_a, mask_b)
    return mask


def _morphology(mask: np.ndarray) -> np.ndarray:
    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
    return mask


# ── morphological skeleton (Zhang-Suen iterative thinning via erosion) ────────

def _skeleton(binary: np.ndarray) -> np.ndarray:
    """Morphological skeleton via iterative erosion + open.

    Avoids scikit-image/ximgproc dependency; works with base opencv-python.
    """
    skel = np.zeros_like(binary)
    img = binary.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while True:
        eroded = cv2.erode(img, kernel)
        temp = cv2.dilate(eroded, kernel)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()
        if cv2.countNonZero(img) == 0:
            break
    return skel


# ── point extraction + ordering ───────────────────────────────────────────────

def _nearest_neighbour_path(
    pts: np.ndarray,  # shape (N, 2) — [row, col]
) -> np.ndarray:
    """Greedy nearest-neighbour traversal from top-most point.

    Returns pts reordered into an approximate polyline. O(N²) — acceptable
    for N ≤ few thousand skeleton pixels.
    """
    if len(pts) == 0:
        return pts

    # Start from the point with smallest row (top = closest to truck on ETS2 map).
    start_idx = int(np.argmin(pts[:, 0]))
    ordered = [start_idx]
    remaining = set(range(len(pts)))
    remaining.discard(start_idx)

    current = start_idx
    while remaining:
        cur_pt = pts[current]
        dists = np.sum((pts[list(remaining)] - cur_pt) ** 2, axis=1)
        nearest_local = int(np.argmin(dists))
        nearest_global = list(remaining)[nearest_local]
        ordered.append(nearest_global)
        remaining.discard(nearest_global)
        current = nearest_global

    return pts[ordered]


def _subsample(pts: np.ndarray, n: int) -> np.ndarray:
    """Evenly subsample `pts` to at most `n` points along the polyline."""
    if len(pts) <= n:
        return pts
    indices = np.round(np.linspace(0, len(pts) - 1, n)).astype(int)
    return pts[indices]


def _largest_component_mask(binary: np.ndarray) -> np.ndarray:
    """Keep only the largest connected component of the binary mask."""
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if n_labels <= 1:
        return binary
    # stats[0] is background; find largest foreground component by area
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest = int(np.argmax(areas)) + 1
    return (labels == largest).astype(np.uint8) * 255


# ── confidence ────────────────────────────────────────────────────────────────

def _compute_confidence(
    roi_area: int,
    mask_px: int,
    skel_px: int,
    n_points: int,
    expected_density: float = 0.002,  # ~0.2% of ROI pixels typical for route line
) -> float:
    """Confidence in [0, 1]:
    - density_score: how much of the ROI is route line (normalised, capped)
    - continuity_score: skeleton length relative to subsampled point count
    - presence_score: binary enough pixels found
    """
    if mask_px == 0 or skel_px == 0 or roi_area == 0:
        return 0.0

    density = mask_px / roi_area
    density_score = min(density / expected_density, 1.0)

    # Continuity: high skeleton pixel count relative to mask means a thin line,
    # which is good (thick blobs are noise).
    ratio = skel_px / max(mask_px, 1)
    continuity_score = min(ratio * 5.0, 1.0)  # ratio ~0.1-0.2 for thin line

    # Presence: require at least MIN_POINTS meaningful points.
    min_points = 5
    presence_score = min(n_points / max(min_points, 1), 1.0)

    return float((density_score * 0.4 + continuity_score * 0.3 + presence_score * 0.3))


# ── main pipeline function ────────────────────────────────────────────────────

def detect_route_line(
    frame_bgr: np.ndarray,
    roi_x: int,
    roi_y: int,
    roi_w: int,
    roi_h: int,
    hsv_lower: tuple[int, int, int],
    hsv_upper: tuple[int, int, int],
    max_points: int = MAX_POINTS,
) -> DetectionResult:
    """Full route-line detection pipeline for one frame.

    Returns DetectionResult with pixel coords (ROI-local) and confidence.
    """
    h, w = frame_bgr.shape[:2]

    # Clamp ROI to frame bounds.
    x1 = max(0, roi_x)
    y1 = max(0, roi_y)
    x2 = min(w, roi_x + roi_w)
    y2 = min(h, roi_y + roi_h)
    if x2 <= x1 or y2 <= y1:
        return DetectionResult(points=[], confidence=0.0)

    roi = frame_bgr[y1:y2, x1:x2]
    roi_area = roi.shape[0] * roi.shape[1]

    mask = _hsv_mask(roi, hsv_lower, hsv_upper)
    mask = _morphology(mask)
    mask = _largest_component_mask(mask)
    mask_px = int(cv2.countNonZero(mask))

    if mask_px < 10:
        return DetectionResult(points=[], confidence=0.0, mask_pixel_count=mask_px)

    skel = _skeleton(mask)
    skel_px = int(cv2.countNonZero(skel))

    if skel_px < 3:
        return DetectionResult(points=[], confidence=0.0, mask_pixel_count=mask_px)

    # Extract pixel coords [row, col].
    ys, xs = np.where(skel > 0)
    pts = np.stack([ys, xs], axis=1).astype(np.float32)  # shape (N, 2)

    # Nearest-neighbour sort — cap at 2000 px before sorting to keep O(N²) fast.
    if len(pts) > 2000:
        step = len(pts) // 2000
        pts = pts[::step]

    ordered = _nearest_neighbour_path(pts)
    subsampled = _subsample(ordered, max_points)

    # Convert to (x, y) tuples (col, row) — matches OpenCV convention.
    point_list = [(float(p[1]), float(p[0])) for p in subsampled]

    confidence = _compute_confidence(roi_area, mask_px, skel_px, len(point_list))

    return DetectionResult(
        points=point_list,
        confidence=confidence,
        mask_pixel_count=mask_px,
        skeleton_pixel_count=skel_px,
    )
