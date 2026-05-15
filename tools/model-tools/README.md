# model-tools

Helpers for preparing TruckPilot ONNX models before deployment to the
`sign-vision` plugin.

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
