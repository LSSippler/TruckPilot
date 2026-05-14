"""Pull reviewed annotations from a CVAT project back as YOLO 1.1.

For each task in the given project we ask CVAT to export YOLO 1.1, unzip it
to a temp dir, and copy the per-image `.txt` files into
`tools/vision-training-collector/data/final/labels/<split>/`, overwriting
whatever's there. Split is taken from the CVAT task's `subset` field
(Train/Validation/Test).

Requires: `pip install cvat-sdk`.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path

from cvat_sdk import make_client

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUT = REPO_ROOT / "tools" / "vision-training-collector" / "data" / "final" / "labels"
STAGE_ROOT = REPO_ROOT / "outputs" / "claude" / "cvat-export"

SUBSET_TO_SPLIT = {"Train": "train", "Validation": "val", "Test": "test"}


def export_task(task, stage: Path) -> Path:
    zip_path = stage / f"task-{task.id}.zip"
    task.export_dataset(format_name="YOLO 1.1", filename=str(zip_path), include_images=False)
    return zip_path


def copy_labels_from_zip(zip_path: Path, out_split_dir: Path, dry_run: bool) -> int:
    """Extract every `.txt` label file from a YOLO 1.1 export zip into out_split_dir."""
    out_split_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    with zipfile.ZipFile(zip_path, "r") as zf:
        for member in zf.namelist():
            if not member.endswith(".txt"):
                continue
            # Skip obj.names / obj.data / train.txt etc.
            base = Path(member).name
            if base in {"obj.names", "obj.data", "train.txt", "test.txt", "val.txt"}:
                continue
            target = out_split_dir / base
            if dry_run:
                print(f"DRY: would write {target}")
            else:
                with zf.open(member) as src, open(target, "wb") as dst:
                    shutil.copyfileobj(src, dst)
            copied += 1
    return copied


def run(args: argparse.Namespace) -> None:
    out_root = Path(args.out).resolve()
    STAGE_ROOT.mkdir(parents=True, exist_ok=True)

    with make_client(host=args.host, credentials=(args.username, args.password)) as client:
        project = client.projects.retrieve(args.project_id)
        tasks = list(project.get_tasks())
        if not tasks:
            sys.exit(f"project {args.project_id} has no tasks")

        with tempfile.TemporaryDirectory(dir=STAGE_ROOT) as tmp:
            tmp_path = Path(tmp)
            total = 0
            for task in tasks:
                subset = task.subset or ""
                split = SUBSET_TO_SPLIT.get(subset)
                if split is None:
                    print(f"skip task {task.id} ({task.name!r}): unknown subset {subset!r}")
                    continue
                zip_path = export_task(task, tmp_path)
                n = copy_labels_from_zip(zip_path, out_root / split, args.dry_run)
                print(f"task {task.id} [{subset}] -> {n} label files into {out_root / split}")
                total += n
            print(f"done: {total} label files written (dry_run={args.dry_run})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--project-id", type=int, required=True)
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--host", default=os.environ.get("CVAT_HOST_URL", "http://localhost:8080"))
    ap.add_argument("--username", default=os.environ.get("CVAT_USER", "admin"))
    ap.add_argument("--password", default=os.environ.get("CVAT_PASS", ""))
    ap.add_argument("--dry-run", action="store_true")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
