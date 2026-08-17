#!/usr/bin/env python3
"""Package the trained can ONNX model as a non-executable ROS OBB bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws/src/so101_top_perception"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

from so101_top_perception import obb_detector  # noqa: E402
from so101_top_perception.detector import file_sha256  # noqa: E402


def load_json(path: Path) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def unused_candidates(
    source_root: Path,
    training_manifest: dict,
) -> tuple[list[Path], list[Path]]:
    used_hashes = {
        item.get("image_sha256")
        for item in training_manifest.get("images", [])
        if isinstance(item, dict)
    }
    positive: list[Path] = []
    negative: list[Path] = []
    for path in source_root.glob("*.png"):
        if file_sha256(path) in used_hashes:
            continue
        target = negative if path.name.startswith("distractors_only_") else positive
        target.append(path)
    key = lambda path: hashlib.sha256(path.name.encode("utf-8")).hexdigest()
    return sorted(positive, key=key), sorted(negative, key=key)


def selected_holdout_entries(
    source_root: Path,
    training_manifest: dict,
    positive_count: int,
    negative_count: int,
) -> list[dict]:
    positive, negative = unused_candidates(source_root, training_manifest)
    if len(positive) < positive_count or len(negative) < negative_count:
        raise ValueError(
            "not enough exact-image holdout candidates: "
            f"positive={len(positive)}/{positive_count} "
            f"negative={len(negative)}/{negative_count}"
        )
    selected = [
        (path, True) for path in positive[:positive_count]
    ] + [
        (path, False) for path in negative[:negative_count]
    ]
    return [
        {
            "path": path.relative_to(source_root).as_posix(),
            "sha256": file_sha256(path),
            "requires_can_obb": requires_can,
        }
        for path, requires_can in selected
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=(
            ROOT
            / "artifacts/can_obb/training/can_yolo11n_obb_30ep/"
            "can_yolo11n_obb.onnx"
        ),
    )
    parser.add_argument(
        "--training-manifest",
        type=Path,
        default=ROOT / "datasets/can_obb_auto_v1/autolabel_manifest.json",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        default=ROOT / "datasets/dual_arm_can_disposal_20260815_measured",
    )
    parser.add_argument(
        "--training-args",
        type=Path,
        default=(
            ROOT
            / "artifacts/can_obb/training/can_yolo11n_obb_30ep/args.yaml"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "artifacts/can_obb/runtime_bundle_v1",
    )
    parser.add_argument("--positive-count", type=int, default=32)
    parser.add_argument("--negative-count", type=int, default=0)
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--iou", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for label, path in (
        ("model", args.model),
        ("training manifest", args.training_manifest),
        ("source root", args.source_root),
        ("training args", args.training_args),
    ):
        if not path.exists():
            raise SystemExit(f"CAN_OBB_BUNDLE_ERROR missing {label}: {path}")
    if args.positive_count <= 0 or args.negative_count < 0:
        raise SystemExit(
            "CAN_OBB_BUNDLE_ERROR positive count must be positive and "
            "negative count must be nonnegative"
        )
    if not 0.0 < args.confidence < 1.0 or not 0.0 < args.iou < 1.0:
        raise SystemExit("CAN_OBB_BUNDLE_ERROR thresholds must be within (0, 1)")

    training_manifest = load_json(args.training_manifest)
    entries = selected_holdout_entries(
        args.source_root,
        training_manifest,
        args.positive_count,
        args.negative_count,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    holdout_path = args.output_dir / "can_correlated_smoke_holdout_manifest.json"
    holdout = {
        "protocol_version": 1,
        "dataset_kind": "can_obb_correlated_smoke_holdout",
        "class_names": ["can"],
        "image_count": len(entries),
        "positive_count": args.positive_count,
        "negative_count": args.negative_count,
        "images": entries,
        "used_for_training": False,
        "deployment_acceptance_permitted": False,
        "limitation": (
            "exact image bytes were not used for training, but frames share "
            "capture sessions with training images; use only for runtime smoke"
        ),
        "selection": "sha256_filename_order_v1",
    }
    holdout_path.write_text(
        json.dumps(holdout, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    model_name = "top_can_yolo11n_obb.onnx"
    deployed_model = args.output_dir / model_name
    shutil.copy2(args.model, deployed_model)
    bundle = {
        "protocol_version": 1,
        "backend": obb_detector.BACKEND_NAME,
        "task": "obb",
        "candidate": True,
        "motion_authorized": False,
        "model": {
            "path": model_name,
            "sha256": file_sha256(deployed_model),
            "format": "onnx",
        },
        "input": {
            "width": 320,
            "height": 320,
            "layout": "NCHW",
            "color_order": "RGB",
            "scale": 1.0 / 255.0,
            "letterbox_value": 114,
        },
        "output": {
            "layout": obb_detector.OUTPUT_LAYOUT,
            "class_names": ["can"],
            "target_class_id": 0,
            "target_class_name": "can",
            "yaw_semantics": obb_detector.YAW_SEMANTICS,
        },
        "thresholds": {
            "confidence": args.confidence,
            "iou": args.iou,
            "maximum_detections": 10,
        },
        "training": {
            "training_manifest_sha256": file_sha256(args.training_manifest),
            "training_args_sha256": file_sha256(args.training_args),
            "holdout_manifest_sha256": file_sha256(holdout_path),
            "holdout_used_for_training": False,
        },
        "evaluation_limitations": {
            "deployment_acceptance_permitted": False,
            "correlated_capture_sessions": True,
            "negative_holdout_available": args.negative_count > 0,
            "false_positive_acceptance_permitted": False,
            "independent_holdout_required_before_unattended_execution": True,
        },
    }
    bundle_path = args.output_dir / "top_can_yolo_obb_bundle.json"
    bundle_path.write_text(
        json.dumps(bundle, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    runtime = obb_detector.OpenCvYoloObbDetector(
        bundle_path,
        expected_holdout_manifest_sha256=file_sha256(holdout_path),
    )
    detections = runtime.infer_detections(
        np.zeros((480, 640, 3), dtype=np.uint8)
    )
    result = {
        "status": "CAN_OBB_DEVELOPMENT_BUNDLE_PASS",
        "motion_authorized": False,
        "bundle_manifest": str(bundle_path),
        "bundle_manifest_sha256": file_sha256(bundle_path),
        "holdout_manifest": str(holdout_path),
        "holdout_manifest_sha256": file_sha256(holdout_path),
        "model": str(deployed_model),
        "model_sha256": file_sha256(deployed_model),
        "opencv_version": cv2.__version__,
        "blank_image_detection_count": len(detections),
    }
    result_path = args.output_dir / "package_result.json"
    result_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        "CAN_OBB_DEVELOPMENT_BUNDLE_PASS "
        f"bundle={bundle_path} "
        f"holdout_sha256={result['holdout_manifest_sha256']} "
        f"blank_detections={len(detections)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
