#!/usr/bin/env python3
"""Repair audited metadata and prepare one sharp representative per capture."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import zipfile

from object_pose_dataset import atomic_write_json, file_sha256, load_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--can-present-capture-id",
        action="append",
        default=[],
        help="capture audited as containing a can despite absent metadata",
    )
    return parser.parse_args()


def repair_can_present_metadata(
    dataset_root: Path,
    manifest: dict,
    capture_ids: list[str],
) -> None:
    entries = {entry["id"]: entry for entry in manifest["captures"]}
    for capture_id in capture_ids:
        if capture_id not in entries:
            raise ValueError(f"capture is not in manifest: {capture_id}")
        entry = entries[capture_id]
        metadata_path = dataset_root / entry["path"]
        document = load_json(metadata_path)
        document["object"]["state"] = "lying"
        document["object"]["position_label"] = "distractors_with_can"
        document["notes"] = "can_with_other_objects_corrected_after_visual_audit"
        atomic_write_json(metadata_path, document)
        entry["object_state"] = "lying"
        entry["position_label"] = "distractors_with_can"
        entry["sha256"] = file_sha256(metadata_path)
    atomic_write_json(dataset_root / "dataset.json", manifest)


def select_representatives(dataset_root: Path, manifest: dict) -> list[dict]:
    selected = []
    for entry in manifest["captures"]:
        metadata_path = dataset_root / entry["path"]
        document = load_json(metadata_path)
        frame = max(document["frames"], key=lambda item: item["sharpness"])
        image_path = dataset_root / frame["file"]
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        if file_sha256(image_path) != frame["sha256"]:
            raise ValueError(f"image hash mismatch: {image_path}")
        selected.append(
            {
                "capture_id": document["capture_id"],
                "source_metadata": metadata_path.name,
                "selected_image": image_path.name,
                "image_sha256": frame["sha256"],
                "sharpness": frame["sharpness"],
                "object_state": document["object"]["state"],
                "position_label": document["object"]["position_label"],
                "lighting": document["conditions"]["lighting"],
                "requires_can_obb": document["object"]["state"] == "lying",
            }
        )
    return selected


def write_package(output_dir: Path, dataset_root: Path, selected: list[dict]) -> None:
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")
    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True)
    for item in selected:
        shutil.copy2(
            dataset_root / item["selected_image"],
            images_dir / item["selected_image"],
        )

    selection_path = output_dir / "selection.json"
    atomic_write_json(
        selection_path,
        {
            "schema_version": 1,
            "source_dataset": str(dataset_root),
            "selection_policy": "maximum_sharpness_frame_per_capture",
            "image_count": len(selected),
            "can_image_count": sum(item["requires_can_obb"] for item in selected),
            "negative_image_count": sum(
                not item["requires_can_obb"] for item in selected
            ),
            "images": selected,
        },
    )

    csv_path = output_dir / "labeling_manifest.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=selected[0].keys())
        writer.writeheader()
        writer.writerows(selected)

    guide = """# Can OBB labeling guide

- Create one label named `can` with class ID 0.
- Use a rotated rectangle tightly around the complete visible can, including rims.
- Images marked `requires_can_obb=true` require one can OBB.
- Images marked `requires_can_obb=false` are negative images; draw no box.
- A 180-degree rotation is equivalent to 0 degrees.
- Export as `Ultralytics YOLO Oriented Bounding Boxes`.
"""
    (output_dir / "LABELING_GUIDE.md").write_text(guide, encoding="utf-8")

    archive_path = output_dir / f"can_obb_cvat_images_{len(selected)}.zip"
    with zipfile.ZipFile(
        archive_path,
        "w",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=6,
    ) as archive:
        for image_path in sorted(images_dir.glob("*.png")):
            archive.write(image_path, arcname=image_path.name)
    atomic_write_json(
        output_dir / "package_manifest.json",
        {
            "schema_version": 1,
            "image_count": len(selected),
            "archive": archive_path.name,
            "archive_sha256": file_sha256(archive_path),
            "selection_sha256": file_sha256(selection_path),
        },
    )


def main() -> int:
    args = parse_args()
    dataset_root = args.dataset_root.resolve()
    output_dir = args.output_dir.resolve()
    manifest = load_json(dataset_root / "dataset.json")
    repair_can_present_metadata(
        dataset_root,
        manifest,
        args.can_present_capture_id,
    )
    selected = select_representatives(dataset_root, manifest)
    write_package(output_dir, dataset_root, selected)
    print(
        "CAN_OBB_CVAT_PACKAGE_PASS "
        f"images={len(selected)} output={output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
