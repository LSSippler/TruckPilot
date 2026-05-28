"""
Classical CV lane detection spike for comparison against UFLD.

Pipeline per frame:
  1. ROI crop (bottom 60% of frame — steering-relevant zone)
  2. Grayscale + Gaussian blur
  3. Canny edge detection
  4. Hough line transform
  5. Cluster into left/right lane candidates, extrapolate to full ROI height
  6. Annotate: green lane lines + red center marker

Usage:
    python test_cv_lanes.py [--frames-dir PATH]
"""

import argparse
import csv
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output", "cv")


def latest_captures_dir():
    base = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "data", "captures"))
    sessions = sorted(glob.glob(os.path.join(base, "*/")))
    return sessions[-1] if sessions else None


def detect_lanes(img_bgr: np.ndarray):
    """
    Run classical CV lane detection on a single frame.
    Returns (lanes, center_offset, debug_img).
      lanes: list of (x_bottom, y_bottom, x_top, y_top) for left and/or right lane
      center_offset: float px (positive = truck right of center), or None
    """
    h, w = img_bgr.shape[:2]

    # ROI: bottom 60%
    roi_top = int(h * 0.40)
    roi = img_bgr[roi_top:h, :]

    # Preprocess
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Triangular mask to ignore car hood and far edges
    mask = np.zeros_like(edges)
    roi_h, roi_w = edges.shape
    pts = np.array([
        [0, roi_h],
        [roi_w, roi_h],
        [int(roi_w * 0.60), int(roi_h * 0.35)],
        [int(roi_w * 0.40), int(roi_h * 0.35)],
    ], np.int32)
    cv2.fillPoly(mask, [pts], 255)
    masked = cv2.bitwise_and(edges, mask)

    # Probabilistic Hough
    lines = cv2.HoughLinesP(masked, rho=1, theta=np.pi / 180,
                            threshold=40, minLineLength=50, maxLineGap=80)

    left_lines  = []
    right_lines = []
    cx = roi_w / 2.0

    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.3:  # too horizontal
                continue
            # Positive slope + right half → right lane; negative + left half → left lane
            mid_x = (x1 + x2) / 2.0
            if slope > 0 and mid_x > cx * 0.6:
                right_lines.append((x1, y1, x2, y2, slope))
            elif slope < 0 and mid_x < cx * 1.4:
                left_lines.append((x1, y1, x2, y2, slope))

    def average_line(line_group, y_bottom, y_top):
        if not line_group:
            return None
        slopes  = [l[4] for l in line_group]
        xs      = [(l[0] + l[2]) / 2 for l in line_group]
        ys      = [(l[1] + l[3]) / 2 for l in line_group]
        slope   = float(np.median(slopes))
        x_mean  = float(np.median(xs))
        y_mean  = float(np.median(ys))
        # y = slope*(x - x_mean) + y_mean  →  x = (y - y_mean)/slope + x_mean
        if abs(slope) < 1e-4:
            return None
        x_bot = int((y_bottom - y_mean) / slope + x_mean)
        x_top = int((y_top    - y_mean) / slope + x_mean)
        return (x_bot, y_bottom, x_top, y_top)

    y_bot = roi_h - 1
    y_top = int(roi_h * 0.35)
    left_lane  = average_line(left_lines,  y_bot, y_top)
    right_lane = average_line(right_lines, y_bot, y_top)

    # Build annotated image (full frame)
    debug = img_bgr.copy()
    lanes_found = []

    def draw_lane(line, color):
        if line is None:
            return
        x_bot, yb, x_top, yt = line
        real_yb = roi_top + yb
        real_yt = roi_top + yt
        cv2.line(debug, (x_bot, real_yb), (x_top, real_yt), color, 4, cv2.LINE_AA)
        lanes_found.append((x_bot, real_yb, x_top, real_yt))

    draw_lane(left_lane,  (0, 255, 0))
    draw_lane(right_lane, (0, 200, 100))

    # Center offset
    center_offset = None
    if left_lane and right_lane:
        # Average x-position at bottom of ROI
        lx = left_lane[0]
        rx = right_lane[0]
        lane_center = (lx + rx) / 2.0
        center_offset = lane_center + 0 - w / 2.0  # x in ROI = x in full frame (same width)
        cx_img  = int(w / 2 + center_offset)
        cy_img  = int(h * 0.85)
        cv2.circle(debug, (cx_img, cy_img), 10, (0, 0, 255), -1)
        cv2.line(debug, (w // 2, cy_img), (cx_img, cy_img), (0, 0, 255), 2)
        cv2.putText(debug, f"offset:{center_offset:+.0f}px", (cx_img + 14, cy_img),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    lane_count = (1 if left_lane else 0) + (1 if right_lane else 0)
    cv2.putText(debug, f"lanes:{lane_count}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    return lanes_found, center_offset, debug


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", default=None)
    args = ap.parse_args()

    frames_dir = args.frames_dir or latest_captures_dir()
    if not frames_dir or not os.path.isdir(frames_dir):
        print(f"ERROR: frames dir not found: {frames_dir}")
        sys.exit(1)

    frames = sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))
    if not frames:
        print(f"ERROR: no JPEG frames in {frames_dir}")
        sys.exit(1)

    print(f"CV spike — {len(frames)} frames in {os.path.abspath(frames_dir)}")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    total_detected = 0
    total_latency  = 0.0
    csv_path = os.path.join(OUTPUT_DIR, "results.csv")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "lane_count", "center_offset_px", "latency_ms", "detected"])

        for frame_path in frames:
            name = os.path.basename(frame_path)
            img  = cv2.imread(frame_path)
            if img is None:
                continue

            t0 = time.perf_counter()
            lanes, offset, debug = detect_lanes(img)
            lat_ms = (time.perf_counter() - t0) * 1000.0

            cv2.imwrite(os.path.join(OUTPUT_DIR, name), debug)

            lc = len(lanes)
            detected = lc >= 1
            total_detected += int(detected)
            total_latency  += lat_ms

            writer.writerow([name, lc,
                             f"{offset:.1f}" if offset is not None else "N/A",
                             f"{lat_ms:.1f}", int(detected)])

            off_str = f"offset={offset:+.0f}px" if offset is not None else "offset=N/A"
            print(f"  {name}: {lc} lanes  {off_str}  {lat_ms:.1f}ms")

    n   = len(frames)
    dr  = total_detected / n * 100 if n else 0
    avg = total_latency  / n if n else 0

    summary = {
        "method": "classical_cv_hough",
        "frames": n,
        "detected": total_detected,
        "detection_rate_pct": round(dr, 1),
        "avg_latency_ms": round(avg, 1),
    }
    with open(os.path.join(OUTPUT_DIR, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n=== CV Spike ===")
    print(f"Detection rate:  {total_detected}/{n}  ({dr:.0f}%)")
    print(f"Avg latency:     {avg:.1f}ms")
    print(f"Outputs:         {OUTPUT_DIR}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
