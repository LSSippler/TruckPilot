"""
UFLD v2 / v1 ONNX inference test on Stage-1 ETS2 capture frames.

UFLD v2 CULane shapes  (ufldv2_culane_res18_320x1600.onnx):
  Input:      [1, 3, 320, 1600]
  loc_row:    [1, 200, 72, 4]   — col_grids x row_anchors x lanes
  loc_col:    [1, 100, 81, 4]   — row_grids x col_anchors x lanes
  exist_row:  [1, 2, 72, 4]     — binary existence per row-anchor
  exist_col:  [1, 2, 81, 4]     — binary existence per col-anchor

UFLD v1 CULane shapes  (ultra_falst_lane_detection_culane_288x800.onnx):
  Input:      input.1  [1, 3, 288, 800]
  Output:     200      [1, 201, 18, 4]  — (griding+1) x row_anchors x lanes

Usage:
    python test_ufld.py [--frames-dir PATH] [--model culane_v2|culane_v1] [--provider DML|CPU]
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
import onnxruntime as ort

MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output", "ufld")

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
LANE_COLORS   = [(0, 255, 0), (0, 200, 100), (100, 255, 0), (0, 255, 200)]

# CULane row anchors for v1 (18 positions, at input height 288)
ROW_ANCHORS_V1_CULANE = [
    121, 131, 141, 150, 160, 170, 180, 189,
    199, 209, 219, 228, 238, 248, 258, 267, 277, 287,
]

KNOWN_MODELS = {
    "culane_v2": "ufldv2_culane_res18_320x1600.onnx",
    "culane_v1": os.path.join("saved_model_culane", "ultra_falst_lane_detection_culane_288x800.onnx"),
}


def find_default_model():
    for key, rel in KNOWN_MODELS.items():
        path = os.path.join(MODELS_DIR, rel)
        if os.path.isfile(path):
            return path, key
    # fallback: any .onnx
    hits = glob.glob(os.path.join(MODELS_DIR, "**", "*.onnx"), recursive=True)
    if hits:
        return hits[0], "unknown"
    return None, None


# ---------------------------------------------------------------------------
# Pre/post-processing helpers
# ---------------------------------------------------------------------------

def letterbox(img_bgr: np.ndarray, target_w: int, target_h: int):
    h, w = img_bgr.shape[:2]
    scale = min(target_w / w, target_h / h)
    nw, nh = int(w * scale), int(h * scale)
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    pad_top  = (target_h - nh) // 2
    pad_left = (target_w - nw) // 2
    canvas[pad_top:pad_top + nh, pad_left:pad_left + nw] = cv2.resize(img_bgr, (nw, nh))
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    normalized = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    tensor = normalized.transpose(2, 0, 1)[np.newaxis].astype(np.float32)
    return tensor, scale, pad_top, pad_left


def unscale_pts(pts, scale, pad_top, pad_left, orig_w, orig_h):
    result = []
    for x, y in pts:
        ox = int(np.clip((x - pad_left) / scale, 0, orig_w - 1))
        oy = int(np.clip((y - pad_top)  / scale, 0, orig_h - 1))
        result.append((ox, oy))
    return result


# ---------------------------------------------------------------------------
# UFLD v2 decoder
# ---------------------------------------------------------------------------

def decode_v2(outputs, input_w: int, input_h: int, exist_thresh: float = 0.5):
    """
    outputs order: loc_row, loc_col, exist_row, exist_col
    loc_row  (1, C, R, L)  — C col-grids, R row-anchors, L lanes
    exist_row (1, 2, R, L) — binary per row-anchor
    Returns list of lane point-lists [(x, y), ...] in input coords.
    """
    loc_row   = outputs[0][0]   # (C, R, L)
    exist_row = outputs[2][0]   # (2, R, L)

    C, R, L = loc_row.shape
    lanes = []

    for l in range(L):
        pts = []
        for r in range(R):
            # existence check via softmax on 2-class exist head
            e = exist_row[:, r, l]
            e_prob = np.exp(e - e.max())
            e_prob /= e_prob.sum()
            if e_prob[1] < exist_thresh:
                continue
            # column position: argmax over C col-grids
            col_logits = loc_row[:, r, l]
            col_idx = int(np.argmax(col_logits))
            x = int(col_idx / C * input_w)
            y = int(r / R * input_h)
            pts.append((x, y))
        if len(pts) >= 2:
            lanes.append(pts)

    return lanes


# ---------------------------------------------------------------------------
# UFLD v1 decoder
# ---------------------------------------------------------------------------

def decode_v1(outputs, input_w: int, input_h: int):
    """
    Single output (1, G+1, R, L): G col-grids + 1 no-lane, R row-anchors, L lanes.
    """
    out = outputs[0][0]   # (G+1, R, L)
    G   = out.shape[0] - 1
    R   = out.shape[1]
    L   = out.shape[2]

    row_anchors = ROW_ANCHORS_V1_CULANE
    lanes = []

    for l in range(L):
        pts = []
        col_idx = np.argmax(out[:, :, l], axis=0)  # (R,)
        for r in range(R):
            if col_idx[r] == G:
                continue
            x = int(col_idx[r] / G * input_w)
            y = row_anchors[r] if r < len(row_anchors) else int(r / R * input_h)
            pts.append((x, y))
        if len(pts) >= 2:
            lanes.append(pts)

    return lanes


# ---------------------------------------------------------------------------
# Visualisation + metrics
# ---------------------------------------------------------------------------

def lanes_center_offset(lanes, frame_w: int):
    if len(lanes) < 2:
        return None, 0.0
    bottoms = sorted(max(l, key=lambda p: p[1])[0] for l in lanes if l)
    if len(bottoms) < 2:
        return None, 0.0
    center = (bottoms[0] + bottoms[-1]) / 2.0
    return center - frame_w / 2.0, min(1.0, len(lanes) / 4.0)


def draw_annotated(img, lanes, center_offset=None):
    out = img.copy()
    for i, lane in enumerate(lanes):
        col = LANE_COLORS[i % len(LANE_COLORS)]
        cv2.polylines(out, [np.array(lane, np.int32)], False, col, 3, cv2.LINE_AA)
        for pt in lane:
            cv2.circle(out, pt, 4, col, -1)
    if center_offset is not None:
        cx = int(img.shape[1] / 2 + center_offset)
        cy = int(img.shape[0] * 0.85)
        cv2.circle(out, (cx, cy), 10, (0, 0, 255), -1)
        cv2.line(out, (img.shape[1] // 2, cy), (cx, cy), (0, 0, 255), 2)
        cv2.putText(out, f"offset:{center_offset:+.0f}px", (cx + 14, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(out, f"lanes:{len(lanes)}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return out


# ---------------------------------------------------------------------------
# Captures helper
# ---------------------------------------------------------------------------

def latest_captures_dir():
    base = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "data", "captures"))
    sessions = sorted(glob.glob(os.path.join(base, "*/")))
    return sessions[-1] if sessions else None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", default=None)
    ap.add_argument("--model", default="culane_v2", choices=list(KNOWN_MODELS.keys()) + ["auto"])
    ap.add_argument("--provider", default="DML", choices=["DML", "CPU"])
    args = ap.parse_args()

    frames_dir = args.frames_dir or latest_captures_dir()
    if not frames_dir or not os.path.isdir(frames_dir):
        print(f"ERROR: frames dir not found: {frames_dir}")
        sys.exit(1)

    frames = sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))
    if not frames:
        print(f"ERROR: no JPEG frames in {frames_dir}")
        sys.exit(1)

    if args.model == "auto":
        model_path, model_key = find_default_model()
    else:
        rel = KNOWN_MODELS[args.model]
        model_path = os.path.join(MODELS_DIR, rel)
        model_key = args.model

    if not model_path or not os.path.isfile(model_path):
        print(f"ERROR: model not found: {model_path}")
        sys.exit(1)

    print(f"Model:   {os.path.basename(model_path)}  ({model_key})")
    print(f"Frames:  {len(frames)} in {os.path.abspath(frames_dir)}")

    providers = (
        [("DmlExecutionProvider", {}), "CPUExecutionProvider"]
        if args.provider == "DML"
        else ["CPUExecutionProvider"]
    )
    sess = ort.InferenceSession(model_path, providers=providers)
    active_ep = sess.get_providers()[0]
    print(f"EP:      {active_ep}")

    inp       = sess.get_inputs()[0]
    input_name = inp.name
    _, _, target_h, target_w = (inp.shape[i] if isinstance(inp.shape[i], int) else d
                                 for i, d in enumerate([1, 3, 320, 1600]))
    # Re-read correctly
    sh = inp.shape
    target_h = sh[2] if isinstance(sh[2], int) and sh[2] > 0 else (320 if "v2" in model_key else 288)
    target_w = sh[3] if isinstance(sh[3], int) and sh[3] > 0 else (1600 if "v2" in model_key else 800)

    print(f"Input:   {input_name}  {target_h}x{target_w}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUTPUT_DIR, "results.csv")

    total_detected = 0
    total_latency  = 0.0

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "lane_count", "center_offset_px", "confidence", "latency_ms", "detected"])

        for frame_path in frames:
            name = os.path.basename(frame_path)
            img  = cv2.imread(frame_path)
            if img is None:
                continue
            oh, ow = img.shape[:2]

            tensor, scale, pad_top, pad_left = letterbox(img, target_w, target_h)

            t0 = time.perf_counter()
            outputs = sess.run(None, {input_name: tensor})
            lat_ms  = (time.perf_counter() - t0) * 1000.0

            if model_key == "culane_v2":
                lanes_inp = decode_v2(outputs, target_w, target_h)
            else:
                lanes_inp = decode_v1(outputs, target_w, target_h)

            lanes = [unscale_pts(l, scale, pad_top, pad_left, ow, oh) for l in lanes_inp]
            offset, conf = lanes_center_offset(lanes, ow)
            detected = len(lanes) >= 1

            annotated = draw_annotated(img, lanes, offset)
            cv2.imwrite(os.path.join(OUTPUT_DIR, name), annotated)

            w.writerow([name, len(lanes),
                        f"{offset:.1f}" if offset is not None else "N/A",
                        f"{conf:.2f}", f"{lat_ms:.1f}", int(detected)])

            total_latency  += lat_ms
            total_detected += int(detected)

            off_str = f"offset={offset:+.0f}px" if offset is not None else "offset=N/A"
            print(f"  {name}: {len(lanes)} lanes  {off_str}  {lat_ms:.1f}ms")

    n   = len(frames)
    dr  = total_detected / n * 100 if n else 0
    avg = total_latency  / n if n else 0

    summary = {
        "model": os.path.basename(model_path),
        "model_key": model_key,
        "provider": active_ep,
        "frames": n,
        "detected": total_detected,
        "detection_rate_pct": round(dr, 1),
        "avg_latency_ms": round(avg, 1),
    }
    with open(os.path.join(OUTPUT_DIR, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n=== UFLD {model_key} ===")
    print(f"Detection rate:  {total_detected}/{n}  ({dr:.0f}%)")
    print(f"Avg latency:     {avg:.1f}ms")
    print(f"Provider:        {active_ep}")
    print(f"Outputs:         {OUTPUT_DIR}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
