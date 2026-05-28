"""
TuSimple UFLD v2 320x800 POC — CPU latency benchmark on recorded ETS2 frames.

Model: ufldv2_tusimple_res18_320x800.onnx
Input:      [1, 3, 320, 800]
loc_row:    [1, 100, 56, 4]  — col_grids x row_anchors x lanes
loc_col:    [1, 100, 41, 4]
exist_row:  [1, 2, 56, 4]
exist_col:  [1, 2, 41, 4]

Usage:
    python test_tusimple.py [--frames-dir PATH] [--provider CPU|DML]
                            [--exist-thresh 0.5] [--output-dir PATH]
"""

import argparse
import datetime
import glob
import json
import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR  = os.path.join(SCRIPT_DIR, "models")
MODEL_NAME  = "ufldv2_tusimple_res18_320x800.onnx"
INPUT_H     = 320
INPUT_W     = 800
CULANE_AVG  = 37.2   # baseline from prior CULane benchmark
CULANE_DR   = 88.9   # detection rate baseline (%)
WARMUP_N    = 3      # frames excluded from latency stats

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
LANE_COLORS   = [(0, 255, 0), (0, 200, 100), (100, 255, 0), (0, 255, 200)]

REPO_ROOT   = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))
CAPTURES_BASE = os.path.join(REPO_ROOT, "data", "captures")
REPORT_PATH = os.path.join(REPO_ROOT, "outputs", "2026-05-23", "stage3", "tusimple_poc_results.md")


# ---------------------------------------------------------------------------
# Frame discovery
# ---------------------------------------------------------------------------

def discover_frames():
    """Return sorted list of all .jpg frames across all capture sessions."""
    sessions = sorted(glob.glob(os.path.join(CAPTURES_BASE, "*/"))
                      , reverse=False)
    frames = []
    for s in sessions:
        frames.extend(sorted(glob.glob(os.path.join(s, "*.jpg"))))
    return frames


# ---------------------------------------------------------------------------
# Preprocessing
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
# Decoder
# ---------------------------------------------------------------------------

def decode_tusimple(outputs, input_w: int, input_h: int, exist_thresh: float = 0.5):
    """
    UFLD v2 TuSimple decoder.
    outputs[0]: loc_row   (1, C, R, L)
    outputs[2]: exist_row (1, 2, R, L)
    Returns list of lane point-lists [(x, y), ...] in input-space coords.
    """
    loc_row   = outputs[0][0]   # (C, R, L)
    exist_row = outputs[2][0]   # (2, R, L)

    C, R, L = loc_row.shape
    lanes = []

    for l in range(L):
        pts = []
        for r in range(R):
            e = exist_row[:, r, l]
            e_prob = np.exp(e - e.max())
            e_prob /= e_prob.sum()
            if e_prob[1] < exist_thresh:
                continue
            col_idx = int(np.argmax(loc_row[:, r, l]))
            x = int(col_idx / C * input_w)
            y = int(r / R * input_h)
            pts.append((x, y))
        if len(pts) >= 2:
            lanes.append(pts)

    return lanes


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def center_offset(lanes, frame_w: int):
    if len(lanes) < 2:
        return None
    bottoms = sorted(max(l, key=lambda p: p[1])[0] for l in lanes if l)
    if len(bottoms) < 2:
        return None
    return (bottoms[0] + bottoms[-1]) / 2.0 - frame_w / 2.0


def draw_annotated(img, lanes, offset=None):
    out = img.copy()
    for i, lane in enumerate(lanes):
        col = LANE_COLORS[i % len(LANE_COLORS)]
        cv2.polylines(out, [np.array(lane, np.int32)], False, col, 3, cv2.LINE_AA)
        for pt in lane:
            cv2.circle(out, pt, 4, col, -1)
    if offset is not None:
        cx = int(img.shape[1] / 2 + offset)
        cy = int(img.shape[0] * 0.85)
        cv2.circle(out, (cx, cy), 10, (0, 0, 255), -1)
        cv2.line(out, (img.shape[1] // 2, cy), (cx, cy), (0, 0, 255), 2)
        cv2.putText(out, f"offset:{offset:+.0f}px", (cx + 14, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(out, f"lanes:{len(lanes)}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def write_report(frame_rows, latencies, detected_count, active_ep, model_name):
    n_measured = len(latencies)
    n_total = len(frame_rows)
    dr = detected_count / n_measured * 100 if n_measured else 0.0
    avg_lat = float(np.mean(latencies)) if latencies else 0.0
    p50_lat = float(np.percentile(latencies, 50)) if latencies else 0.0
    p95_lat = float(np.percentile(latencies, 95)) if latencies else 0.0

    delta_avg = avg_lat - CULANE_AVG

    go = avg_lat < 25.0 and dr >= 85.0
    verdict = "**GO**" if go else "**NO-GO**"
    if go:
        reason = (f"Avg latenz {avg_lat:.1f}ms < 25ms Ziel, "
                  f"Detection-Rate {dr:.1f}% >= 85% Ziel. "
                  f"Phase 5.29-C.2 Migration empfohlen.")
    else:
        reasons = []
        if avg_lat >= 25.0:
            reasons.append(f"Latenz {avg_lat:.1f}ms > 25ms")
        if dr < 85.0:
            reasons.append(f"Detection-Rate {dr:.1f}% < 85%")
        reason = ", ".join(reasons) + ". TuSimple-Migration nicht empfohlen."

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    lines = [
        f"# TuSimple UFLD v2 320x800 — POC Results",
        f"Generated: {now}",
        f"",
        f"## Latenz-Statistik",
        f"",
        f"| Metrik      | TuSimple          | CULane (Baseline) | Δ          |",
        f"|-------------|-------------------|-------------------|------------|",
        f"| Avg         | {avg_lat:.1f}ms            | {CULANE_AVG}ms           | {delta_avg:+.1f}ms     |",
        f"| P50         | {p50_lat:.1f}ms            | —                 | —          |",
        f"| P95         | {p95_lat:.1f}ms            | —                 | —          |",
        f"| Warm-up (3) | excl.             | —                 | —          |",
        f"",
        f"## Detection-Rate",
        f"",
        f"| Metrik           | TuSimple        | CULane (Baseline) | Ziel   |",
        f"|------------------|-----------------|-------------------|--------|",
        f"| Detected / Total | {detected_count}/{n_measured}             | —                 | —      |",
        f"| Detection Rate   | {dr:.1f}%          | {CULANE_DR}%            | ≥85%   |",
        f"",
        f"## Frame-Detail",
        f"",
        f"| Frame | Latenz (ms) | Lanes | Offset (px) | Detected |",
        f"|-------|-------------|-------|-------------|----------|",
    ]

    for row in frame_rows:
        warmup_tag = " *(warm-up)*" if row["warmup"] else ""
        offset_str = f"{row['offset']:+.0f}" if row["offset"] is not None else "N/A"
        det_str = "YES" if row["detected"] else "no"
        lat_str = f"{row['latency_ms']:.1f}{warmup_tag}"
        lines.append(
            f"| {row['name']} | {lat_str} | {row['lanes']} | {offset_str} | {det_str} |"
        )

    lines += [
        f"",
        f"## Konfiguration",
        f"",
        f"| Parameter     | Wert                                  |",
        f"|---------------|---------------------------------------|",
        f"| Model         | {model_name}        |",
        f"| Provider      | {active_ep}                |",
        f"| Input Size    | {INPUT_H}x{INPUT_W}                           |",
        f"| Frames total  | {n_total} ({WARMUP_N} warm-up, {n_measured} measured) |",
        f"",
        f"## Empfehlung",
        f"",
        f"{verdict} — {reason}",
        f"",
    ]

    if go:
        lines += [
            f"### Phase 5.29-C.2 Migration-Aufwand",
            f"",
            f"- **Rust plugin**: `crates/plugins/lane-keeper/` — ONNX-Session auf TuSimple-Pfad umstellen",
            f"- **Input-Shape**: `[1, 3, 320, 800]` statt `[1, 3, 320, 1600]`",
            f"- **Decoder**: Shapes werden dynamisch gelesen — kein Hardcoding nötig",
            f"- **Geschätzt**: ~2h Anpassung + 1h Validierung",
            f"",
        ]

    os.makedirs(os.path.dirname(REPORT_PATH), exist_ok=True)
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    return avg_lat, p50_lat, p95_lat, dr, go


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="TuSimple UFLD v2 320x800 POC benchmark")
    ap.add_argument("--frames-dir", default=None,
                    help="Directory with .jpg frames (default: auto-discover all captures)")
    ap.add_argument("--provider", default="CPU", choices=["CPU", "DML"],
                    help="ONNX Execution Provider (default: CPU for fair comparison)")
    ap.add_argument("--exist-thresh", type=float, default=0.5,
                    help="Lane existence probability threshold (default: 0.5)")
    ap.add_argument("--output-dir", default=os.path.join(SCRIPT_DIR, "output", "tusimple"),
                    help="Directory for annotated frame output")
    args = ap.parse_args()

    # Model
    model_path = os.path.join(MODELS_DIR, MODEL_NAME)
    if not os.path.isfile(model_path):
        print(f"ERROR: model not found: {model_path}")
        print(f"  Expected: {MODEL_NAME} in {MODELS_DIR}")
        sys.exit(1)

    # Frames
    if args.frames_dir:
        if not os.path.isdir(args.frames_dir):
            print(f"ERROR: frames dir not found: {args.frames_dir}")
            sys.exit(1)
        frames = sorted(glob.glob(os.path.join(args.frames_dir, "*.jpg")))
    else:
        frames = discover_frames()

    if not frames:
        print(f"ERROR: no .jpg frames found (captures base: {CAPTURES_BASE})")
        sys.exit(1)

    print(f"Model:    {MODEL_NAME}")
    print(f"Frames:   {len(frames)} total ({WARMUP_N} warm-up, {len(frames) - WARMUP_N} measured)")
    print(f"Thresh:   exist_thresh={args.exist_thresh}")

    # Session
    providers = (
        [("DmlExecutionProvider", {}), "CPUExecutionProvider"]
        if args.provider == "DML"
        else ["CPUExecutionProvider"]
    )
    sess = ort.InferenceSession(model_path, providers=providers)
    active_ep = sess.get_providers()[0]
    print(f"EP:       {active_ep}")

    # Validate input shape
    inp = sess.get_inputs()[0]
    expected_shape = (1, 3, INPUT_H, INPUT_W)
    actual_shape = tuple(inp.shape)
    if any(isinstance(d, int) and d > 0 and d != e
           for d, e in zip(actual_shape, expected_shape)):
        print(f"ERROR: shape mismatch")
        print(f"  Expected: {expected_shape}")
        print(f"  Actual:   {actual_shape}")
        sys.exit(1)
    print(f"Input:    {inp.name}  {actual_shape}")

    # Validate output shapes
    for out in sess.get_outputs():
        print(f"Output:   {out.name}  {tuple(out.shape)}")

    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Out dir:  {args.output_dir}")
    print()

    latencies = []
    frame_rows = []
    detected_count = 0

    for i, frame_path in enumerate(frames):
        name = os.path.basename(frame_path)
        is_warmup = i < WARMUP_N

        img = cv2.imread(frame_path)
        if img is None:
            print(f"  {name}: SKIP (unreadable)")
            continue
        oh, ow = img.shape[:2]

        tensor, scale, pad_top, pad_left = letterbox(img, INPUT_W, INPUT_H)

        t0 = time.perf_counter()
        outputs = sess.run(None, {inp.name: tensor})
        lat_ms = (time.perf_counter() - t0) * 1000.0

        lanes_inp = decode_tusimple(outputs, INPUT_W, INPUT_H, args.exist_thresh)
        lanes = [unscale_pts(l, scale, pad_top, pad_left, ow, oh) for l in lanes_inp]
        offset = center_offset(lanes, ow)
        detected = len(lanes) >= 1

        annotated = draw_annotated(img, lanes, offset)
        cv2.imwrite(os.path.join(args.output_dir, name), annotated)

        warmup_tag = " [warm-up]" if is_warmup else ""
        off_str = f"offset={offset:+.0f}px" if offset is not None else "offset=N/A"
        det_tag = "[DETECTED]" if detected else "[MISS]"
        print(f"  {name}: {len(lanes)} lanes  {off_str}  {lat_ms:.1f}ms{warmup_tag}  {det_tag}")

        frame_rows.append({
            "name": name,
            "latency_ms": lat_ms,
            "lanes": len(lanes),
            "offset": offset,
            "detected": detected,
            "warmup": is_warmup,
        })

        if not is_warmup:
            latencies.append(lat_ms)
            if detected:
                detected_count += 1

    if not latencies:
        print("\nERROR: no measured frames (all were warm-up or unreadable)")
        sys.exit(1)

    avg_lat, p50_lat, p95_lat, dr, go = write_report(
        frame_rows, latencies, detected_count, active_ep, MODEL_NAME
    )

    n_measured = len(latencies)
    dr_raw = detected_count / n_measured * 100

    print()
    print("=" * 56)
    print(f"=== TuSimple UFLD v2 320x800 — POC Results ===")
    print("=" * 56)
    print(f"Frames total:      {len(frames)}  ({WARMUP_N} warm-up, {n_measured} measured)")
    print(f"Detection rate:    {detected_count}/{n_measured}  ({dr_raw:.1f}%)  target: >=85%")
    print(f"Avg latency:       {avg_lat:.1f}ms          target: <=25ms")
    print(f"P50 latency:       {p50_lat:.1f}ms")
    print(f"P95 latency:       {p95_lat:.1f}ms")
    print(f"vs CULane avg:     {CULANE_AVG}ms  (delta {avg_lat - CULANE_AVG:+.1f}ms)")
    print(f"Provider:          {active_ep}")
    print(f"Model:             {MODEL_NAME}")
    print()
    print(f"Report:  {REPORT_PATH}")
    verdict = "GO" if go else "NO-GO"
    print(f"Verdict: {verdict}")

    # Summary copy per CC3 spec
    cc3_out = os.path.join(REPO_ROOT, "outputs", "claude", "cc3_tusimple_poc.txt")
    os.makedirs(os.path.dirname(cc3_out), exist_ok=True)
    with open(cc3_out, "w", encoding="utf-8") as f:
        f.write(f"CC3 TuSimple POC — {datetime.datetime.now()}\n")
        f.write(f"Model:             {MODEL_NAME}\n")
        f.write(f"Provider:          {active_ep}\n")
        f.write(f"Frames:            {len(frames)} ({WARMUP_N} warm-up, {n_measured} measured)\n")
        f.write(f"Avg latency:       {avg_lat:.1f}ms  (target <25ms)\n")
        f.write(f"P50 latency:       {p50_lat:.1f}ms\n")
        f.write(f"P95 latency:       {p95_lat:.1f}ms\n")
        f.write(f"Detection rate:    {detected_count}/{n_measured}  ({dr_raw:.1f}%)  (target >=85%)\n")
        f.write(f"CULane baseline:   {CULANE_AVG}ms avg, {CULANE_DR}% detection\n")
        f.write(f"Verdict:           {verdict}\n")
        f.write(f"Full report:       {REPORT_PATH}\n")
    print(f"Summary: {cc3_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
