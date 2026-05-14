"""Upload one TruckPilot split into a CVAT project as a task with pre-labels.

Reads `tools/vision-training-collector/data/final/{images,labels}/<split>/`
(produced by the vision-training-collector exporter) and creates a task in
the given CVAT project:

  1. Pack the split's images + YOLO `.txt` files into a YOLO 1.1 zip in
     `outputs/claude/cvat-import/<task-name>.zip`. The zip layout is what
     CVAT's YOLO 1.1 importer expects: `obj.data`, `obj.names`,
     `train.txt`, `obj_train_data/<frame>.{jpg,txt}`.
  2. Create a CVAT task in the project with the matching `subset`
     (Train/Validation/Test).
  3. Upload images + initial YOLO annotations.

Requires: `pip install cvat-sdk pyyaml`.

The script asserts that the CVAT project's labels match the 15-class order
in `class_mapping.yaml`. If they don't, it bails before uploading — wrong
class IDs in a label file silently mislabel hundreds of boxes.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import zipfile
from pathlib import Path

import yaml
from cvat_sdk import make_client
from cvat_sdk.core.proxies.tasks import ResourceType

REPO_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = REPO_ROOT / "tools" / "vision-training-collector" / "data" / "final"
CLASS_MAPPING = REPO_ROOT / "tools" / "vision-training-collector" / "class_mapping.yaml"
STAGE_ROOT = REPO_ROOT / "outputs" / "claude" / "cvat-import"

SPLIT_TO_SUBSET = {"train": "Train", "val": "Validation", "test": "Test"}


def load_expected_classes() -> list[str]:
    data = yaml.safe_load(CLASS_MAPPING.read_text(encoding="utf-8"))
    classes = data["truckpilot_classes"]
    return [classes[i] for i in range(len(classes))]


def assert_project_labels_match(project, expected: list[str]) -> None:
    actual = [lbl.name for lbl in project.get_labels()]
    if actual != expected:
        sys.exit(
            "CVAT project labels do not match class_mapping.yaml.\n"
            f"  expected: {expected}\n"
            f"  actual:   {actual}\n"
            "Fix the project labels (order matters — they map to YOLO class IDs)."
        )


def build_yolo_zip(split: str, task_name: str, classes: list[str]) -> Path:
    """Pack images + labels for one split into a YOLO 1.1 zip CVAT can ingest."""
    images_src = DATASET_ROOT / "images" / split
    labels_src = DATASET_ROOT / "labels" / split
    if not images_src.is_dir():
        sys.exit(f"missing image dir: {images_src}")
    if not labels_src.is_dir():
        sys.exit(f"missing label dir: {labels_src}")

    stage = STAGE_ROOT / task_name
    if stage.exists():
        shutil.rmtree(stage)
    obj_data_dir = stage / "obj_train_data"
    obj_data_dir.mkdir(parents=True)

    image_files = sorted(p for p in images_src.iterdir() if p.suffix.lower() == ".jpg")
    if not image_files:
        sys.exit(f"no .jpg files in {images_src}")

    train_list_lines: list[str] = []
    for img in image_files:
        shutil.copy2(img, obj_data_dir / img.name)
        label = labels_src / f"{img.stem}.txt"
        # CVAT requires a label file per image even if empty.
        (obj_data_dir / f"{img.stem}.txt").write_text(
            label.read_text(encoding="utf-8") if label.exists() else "",
            encoding="utf-8",
        )
        train_list_lines.append(f"data/obj_train_data/{img.name}")

    (stage / "obj.names").write_text("\n".join(classes) + "\n", encoding="utf-8")
    (stage / "obj.data").write_text(
        "\n".join(
            [
                f"classes = {len(classes)}",
                "train = data/train.txt",
                "names = data/obj.names",
                "backup = backup/",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (stage / "train.txt").write_text("\n".join(train_list_lines) + "\n", encoding="utf-8")

    zip_path = STAGE_ROOT / f"{task_name}.zip"
    if zip_path.exists():
        zip_path.unlink()
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for path in stage.rglob("*"):
            if path.is_file():
                zf.write(path, path.relative_to(stage))
    return zip_path


def upload(args: argparse.Namespace) -> None:
    expected = load_expected_classes()

    host = args.host
    with make_client(host=host, credentials=(args.username, args.password)) as client:
        project = client.projects.retrieve(args.project_id)
        assert_project_labels_match(project, expected)

        zip_path = build_yolo_zip(args.split, args.task_name, expected)
        images_src = DATASET_ROOT / "images" / args.split
        image_files = sorted(str(p) for p in images_src.iterdir() if p.suffix.lower() == ".jpg")

        task = client.tasks.create_from_data(
            spec={
                "name": args.task_name,
                "project_id": args.project_id,
                "subset": SPLIT_TO_SUBSET[args.split],
            },
            resource_type=ResourceType.LOCAL,
            resources=image_files,
        )
        print(f"created task id={task.id} name={args.task_name} frames={len(image_files)}")

        task.import_annotations(format_name="YOLO 1.1", filename=str(zip_path))
        print(f"imported pre-labels from {zip_path.name}")
        print(f"open: {host}/tasks/{task.id}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--project-id", type=int, required=True)
    ap.add_argument("--task-name", required=True)
    ap.add_argument("--split", choices=("train", "val", "test"), required=True)
    ap.add_argument("--host", default=os.environ.get("CVAT_HOST_URL", "http://localhost:8080"))
    ap.add_argument("--username", default=os.environ.get("CVAT_USER", "admin"))
    ap.add_argument("--password", default=os.environ.get("CVAT_PASS", ""))
    upload(ap.parse_args())


if __name__ == "__main__":
    main()
