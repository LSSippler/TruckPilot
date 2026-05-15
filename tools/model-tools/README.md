# model-tools

Helpers for preparing TruckPilot ONNX models before deployment to the
`sign-vision` plugin.

## extract_sign_crops.py — crop SpeedLimitSign bboxes from saved frames

Reads a saved-frames directory (from `vision-pipeline-capture --save-frames-dir`)
plus an NDJSON dump of `sign.detections.last_n` and writes one cropped PNG
per detection of class `SpeedLimitSign`. Used by Phase 6.5h to regenerate
real-image NCC templates for `SpeedMapper` (the synthetic 5×7 bitmap
templates only match ~12% of real signs).

### Workflow

```powershell
# 1. Drive a fresh ~5 min test with frames mirrored to disk:
python -m vision_pipeline_capture start --save-frames-dir capture_frames\

# 2. Dump the detection log from the daemon's blackboard:
.\target\release\blackboard-query.exe --keys sign.detections.last_n `
    > outputs\sign_detections_dump.txt

# 3. Extract crops:
python tools\model-tools\extract_sign_crops.py `
    --frames-dir capture_frames\ `
    --ndjson outputs\sign_detections_dump.txt `
    --out-dir tools\model-tools\sign_crops\

# 4. Hand-label: in sign_crops/, group crops into <kmh>/ subdirs:
#      sign_crops/40/<id>.png, sign_crops/60/<id>.png, ...
#    Delete out-of-frame / unreadable crops. The _mapped/ subdir
#    serves as a labeled-by-SpeedMapper reference; check it but do
#    not feed it to the template builder (build_sign_templates skips
#    non-numeric subdir names).
```

### Why a frames mirror is needed

The blackboard NDJSON only carries `frame_id` (and bbox + confidence),
no timestamp. There is no way to align the daemon's logical frame_ids
to an OBS or external video recording without re-running with the
`--save-frames-dir` flag, which writes `<seq>.jpg` per published frame
(NDJSON `f * 2 == seq`).

## build_sign_templates.py — generate real-image NCC templates

Reads `tools/model-tools/sign_crops/<kmh>/*.png` (output of the labeling
step above) and writes `crates/plugins/sign-vision/src/speed_templates_real.rs`
with one embedded `[u8; 1024]` per crop. The Rust `SpeedMapper` tries
real templates first and falls back to synthetic 5×7-bitmap-font
templates for any km/h with no real exemplars.

```powershell
python tools\model-tools\build_sign_templates.py
# Found 4 km/h classes, N crops total: ...
# Wrote crates/plugins/sign-vision/src/speed_templates_real.rs

cargo build -p truckpilot-plugin-sign-vision --release
cargo xtask copy-plugins
# Restart daemon, run live test, check sign.class.SpeedLimitSign.km_<n>.total
```

Re-run after every fresh labeling pass. The generated `.rs` file is
checked in; an empty placeholder ships when no labeled crops exist.

## quantize_fp16.py — FP32 → FP16

Converts a YOLOv8s model (or any ONNX float model) to FP16 compute,
keeping inputs/outputs as FP32 so the Rust plugin's preprocess and
postprocess paths work unchanged.

### Why

`sign-vision` runs ONNX Runtime + DirectML on the AMD RX 7800 XT.
With FP32 weights, inference is 110–130 ms/frame; this exceeds the
50 Hz tick budget and starves the watchdog heartbeat. FP16 typically
halves both model size (~22 MB → ~11 MB) and latency (target <30 ms)
on RDNA3 with negligible mAP loss for YOLOv8s.

### Setup (one-time)

```powershell
# In a venv or wherever you keep Python tooling for this repo:
pip install onnx onnxconverter-common
```

### Run

```powershell
python tools/model-tools/quantize_fp16.py models/truckpilot-yolov8s-v2/best.onnx
# Writes models/truckpilot-yolov8s-v2/best_fp16.onnx
```

### After retraining

Whenever a new `best.onnx` lands in `models/truckpilot-yolov8s-v2/` or
`models/truckpilot-yolov8s-v3/`:

1. Re-run the script against the new FP32 file.
2. If you bumped the model directory (v2 → v3), update
   `DEFAULT_MODEL_PATH` in `crates/plugins/sign-vision/src/lib.rs`.
3. `cargo build --release -p truckpilot-plugin-sign-vision` and
   `cargo xtask copy-plugins`.
4. Restart the daemon and verify `sign.last_inference_ms` in the
   blackboard. Expected: 20–60 ms (DirectML + FP16). If still
   >100 ms, check `sign.onnx.provider` — DirectML may have failed
   silently and fallen back to CPU.

### Verification

There is no automated mAP regression suite for the quantized model
yet; in practice FP16 keeps YOLOv8s within ~0.5% of the FP32 mAP.
If you suspect quality regression, re-export the model from
Ultralytics with the `half=True` flag and compare against this script's
output — they should be near-identical.
