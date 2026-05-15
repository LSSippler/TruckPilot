"""Extract speed-limit-sign crops from saved frames + sign-vision NDJSON.

Background
----------
Phase 6.5h investigation: the synthetic-bitmap-font templates in
`SpeedMapper` only match ~12% of YOLO's `SpeedLimitSign` detections in
real ETS2 footage. To regenerate the templates from real signs, we need
labeled crops of the bboxes the daemon actually produces.

Inputs
------
- A frames directory written by ``vision-pipeline-capture --save-frames-dir``.
  Filenames are ``<seq>.jpg`` where ``seq`` is the SHM writer's sequence
  number. NDJSON ``f`` (logical frame_id) maps to ``f * 2.jpg``.
- An NDJSON dump of ``sign.detections.last_n`` from the blackboard,
  e.g. produced by ``blackboard-query --keys sign.detections.last_n``.
  The dump may carry a one-line header ("1 key(s):" + key name); the
  parser skips any non-JSON lines.

Output
------
- ``<out-dir>/<frame_id>_<x1>_<y1>_<x2>_<y2>.png`` — one cropped PNG per
  matching detection. Only writes detections where ``c==11`` (SpeedLimitSign)
  AND ``k`` is missing (= unmapped by SpeedMapper). Mapped detections are
  also written into ``<out-dir>/_mapped/`` so the labeler can compare.

The user then groups crops by hand into ``<labeled-out>/<kmh>/<id>.png``
which feeds the template-generation step.

Usage
-----
    python extract_sign_crops.py \\
        --frames-dir   path/to/frames \\
        --ndjson       outputs/sign_detections_dump.txt \\
        --out-dir      tools/model-tools/sign_crops

Exit codes
----------
- 0 : at least one crop written
- 1 : no matching detections in NDJSON
- 2 : input file/dir missing
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


# Class id of `SpeedLimitSign` in CLASS_NAMES (Rust side).
SPEED_LIMIT_CLASS_ID = 11

# How a logical frame_id maps to the saved JPEG sequence number on disk.
# vision-frame-source halves the SHM seq to derive its logical id, so the
# inverse here is `* 2`.
def seq_for_frame_id(f: int) -> int:
    return f * 2


def parse_ndjson(path: Path) -> list[dict]:
    """Load NDJSON entries, tolerant of leading non-JSON header lines."""
    out: list[dict] = []
    text = path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        line = line.strip()
        if not line or not line.startswith("{"):
            # blackboard-query prepends "1 key(s):" + "key = " before the
            # actual JSON. Skip anything that doesn't look like a JSON
            # object opener. Also handle "key = {...}" lines.
            if " = {" in line:
                line = line.split(" = ", 1)[1]
            else:
                continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            # Best-effort: skip malformed lines without aborting.
            continue
    return out


def crop_and_save(
    frames_dir: Path,
    entry: dict,
    out_path: Path,
) -> bool:
    """Crop one bbox from the corresponding saved frame and write PNG.
    Returns True on success, False if the source frame is missing or the
    bbox is empty after clamping."""
    try:
        from PIL import Image
    except ImportError:
        sys.stderr.write(
            "Pillow not installed. Install with: pip install Pillow\n"
        )
        sys.exit(2)

    frame_id = int(entry["f"])
    seq = seq_for_frame_id(frame_id)
    src = frames_dir / f"{seq}.jpg"
    if not src.is_file():
        return False

    bbox = entry.get("b") or [0, 0, 0, 0]
    if len(bbox) != 4:
        return False
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)

    with Image.open(src) as im:
        w, h = im.size
        # Clamp to frame and reject degenerate boxes.
        x1 = max(0, min(x1, w - 1))
        y1 = max(0, min(y1, h - 1))
        x2 = max(0, min(x2, w))
        y2 = max(0, min(y2, h))
        if x2 - x1 < 4 or y2 - y1 < 4:
            return False
        crop = im.crop((x1, y1, x2, y2))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        crop.save(out_path)
    return True


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--frames-dir", type=Path, required=True,
                   help="Dir of <seq>.jpg files from --save-frames-dir.")
    p.add_argument("--ndjson", type=Path, required=True,
                   help="NDJSON dump of sign.detections.last_n.")
    p.add_argument("--out-dir", type=Path, required=True,
                   help="Output dir for cropped PNGs.")
    args = p.parse_args()

    if not args.frames_dir.is_dir():
        sys.stderr.write(f"frames-dir not found: {args.frames_dir}\n")
        sys.exit(2)
    if not args.ndjson.is_file():
        sys.stderr.write(f"ndjson not found: {args.ndjson}\n")
        sys.exit(2)

    entries = parse_ndjson(args.ndjson)
    speed_limits = [e for e in entries if e.get("c") == SPEED_LIMIT_CLASS_ID]
    unmapped = [e for e in speed_limits if "k" not in e]
    mapped = [e for e in speed_limits if "k" in e]

    print(
        f"NDJSON: {len(entries)} entries, "
        f"{len(speed_limits)} SpeedLimitSign "
        f"({len(unmapped)} unmapped, {len(mapped)} mapped)"
    )

    written = 0
    skipped_missing_frame = 0
    for e in unmapped:
        bbox = e.get("b") or [0, 0, 0, 0]
        x1, y1, x2, y2 = (int(round(v)) for v in bbox)
        out_path = args.out_dir / f"{e['f']}_{x1}_{y1}_{x2}_{y2}.png"
        if crop_and_save(args.frames_dir, e, out_path):
            written += 1
        else:
            skipped_missing_frame += 1

    # Also save mapped crops so the labeler can use them as visual
    # reference + sanity check that the "successful" mappings are right.
    mapped_dir = args.out_dir / "_mapped"
    mapped_written = 0
    for e in mapped:
        bbox = e.get("b") or [0, 0, 0, 0]
        x1, y1, x2, y2 = (int(round(v)) for v in bbox)
        kmh = int(e["k"])
        out_path = mapped_dir / f"{kmh}_{e['f']}_{x1}_{y1}_{x2}_{y2}.png"
        if crop_and_save(args.frames_dir, e, out_path):
            mapped_written += 1

    print(
        f"wrote {written} unmapped crops -> {args.out_dir}\n"
        f"wrote {mapped_written} mapped reference crops -> {mapped_dir}\n"
        f"skipped (frame missing or bbox degenerate): {skipped_missing_frame}"
    )

    if written == 0 and mapped_written == 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
