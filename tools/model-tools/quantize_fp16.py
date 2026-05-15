"""Convert a YOLOv8s ONNX model from FP32 to FP16 compute.

Background
----------
The sign-vision plugin loads a YOLOv8s ONNX model and runs inference via
ONNX Runtime + DirectML on the AMD RX 7800 XT. With the default FP32
weights (~22 MB) inference takes 110-130 ms per frame, which exceeds the
50 Hz tick budget and starves the watchdog heartbeat. Converting the
compute graph to FP16 typically halves both model size (~11 MB) and
latency (target <30 ms) on RDNA3 GPUs while keeping detection quality
within ~0.5% mAP for YOLOv8s.

What this script does
---------------------
- Loads the FP32 ONNX model
- Converts internal weights and intermediate tensors to FP16 via
  `onnxconverter_common.float16.convert_float_to_float16`
- Keeps the model's INPUT and OUTPUT tensors as FP32 so the plugin's
  preprocess (`to_nchw` -> Tensor::<f32>) and postprocess
  (`outputs["output0"].try_extract_tensor::<f32>`) keep working
  unchanged - no Rust code change for the IO contract
- Writes the result next to the input with a `_fp16.onnx` suffix

Why keep IO as FP32
-------------------
The Rust side allocates `Tensor::<f32>` for the input and unpacks
`f32` from the output. Converting IO to FP16 would silently break
preprocess (NCHW tensor type mismatch) and postprocess (zero-cost
reinterpret of f16 bytes as f32 = garbage anchors). The cost of FP32
IO is one extra cast per inference at the GPU boundary, which is
negligible compared to the inference itself.

Install
-------
    pip install onnx onnxconverter-common

Usage
-----
    python quantize_fp16.py <path-to-fp32.onnx>
    python quantize_fp16.py <path-to-fp32.onnx> --output <out.onnx>

Re-run after each model retrain. The output file is what
`sign-vision`'s `DEFAULT_MODEL_PATH` points to.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def convert(input_path: Path, output_path: Path) -> None:
    try:
        import onnx
        from onnxconverter_common import float16
    except ImportError as e:
        sys.stderr.write(
            f"Missing dependency: {e}\n"
            "Install with: pip install onnx onnxconverter-common\n"
        )
        sys.exit(2)

    if not input_path.is_file():
        sys.stderr.write(f"Input model not found: {input_path}\n")
        sys.exit(1)

    in_size_mb = input_path.stat().st_size / (1024 * 1024)
    print(f"Loading {input_path} ({in_size_mb:.1f} MB)...")
    model = onnx.load(str(input_path))

    print("Converting to FP16 (keeping FP32 inputs/outputs for plugin IO compat)...")
    model_fp16 = float16.convert_float_to_float16(
        model,
        keep_io_types=True,
        # Skip ops that are known to be unstable in FP16 on some EPs.
        # Empty tuple = convert everything float16.convert_float_to_float16
        # considers safe by default.
        op_block_list=None,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model_fp16, str(output_path))

    out_size_mb = output_path.stat().st_size / (1024 * 1024)
    ratio = out_size_mb / in_size_mb if in_size_mb > 0 else 0.0
    print(f"Wrote {output_path} ({out_size_mb:.1f} MB, {ratio * 100:.0f}% of original)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path, help="Path to FP32 .onnx model")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output path. Default: <input-stem>_fp16.onnx next to the input.",
    )
    args = parser.parse_args()

    output = args.output or args.input.with_name(args.input.stem + "_fp16.onnx")
    convert(args.input, output)


if __name__ == "__main__":
    main()
