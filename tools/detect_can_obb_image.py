#!/usr/bin/env python3
"""Detect exactly one can OBB in an image using a trained Ultralytics model.

This desktop/offline tool creates no ROS entities and never authorizes robot
motion. It converts an OBB into the image-space center, undirected long-axis
yaw, and the perpendicular gripper-closing yaw needed by the later MoveIt
planning stage.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np


STATUS = "CAN_OBB_IMAGE_DETECTION_PASS"
YAW_SEMANTICS = "undirected_long_axis_modulo_pi"


def normalize_axis_yaw_deg(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("axis yaw must be finite")
    normalized = value % 180.0
    return 0.0 if math.isclose(normalized, 180.0) else normalized


def order_box_points(points: np.ndarray) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    if values.shape != (4, 2) or not np.all(np.isfinite(values)):
        raise ValueError("OBB corners must be a finite 4x2 array")
    center = np.mean(values, axis=0)
    angles = np.arctan2(values[:, 1] - center[1], values[:, 0] - center[0])
    return values[np.argsort(angles)]


def pose_from_corners(
    points: np.ndarray,
    confidence: float,
) -> dict[str, Any]:
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and within 0..1")
    ordered = order_box_points(points)
    if abs(float(cv2.contourArea(ordered.astype(np.float32)))) <= 1.0:
        raise ValueError("OBB area is too small")
    edges = np.roll(ordered, -1, axis=0) - ordered
    lengths = np.linalg.norm(edges, axis=1)
    long_index = int(np.argmax(lengths))
    long_edge = edges[long_index]
    long_side = float((lengths[long_index] + lengths[(long_index + 2) % 4]) / 2.0)
    short_index = (long_index + 1) % 4
    short_side = float(
        (lengths[short_index] + lengths[(short_index + 2) % 4]) / 2.0
    )
    long_axis_yaw = normalize_axis_yaw_deg(
        math.degrees(math.atan2(float(long_edge[1]), float(long_edge[0])))
    )
    closing_yaw = normalize_axis_yaw_deg(long_axis_yaw + 90.0)
    center = np.mean(ordered, axis=0)
    return {
        "class_name": "can",
        "confidence": float(confidence),
        "center_px": [float(center[0]), float(center[1])],
        "corners_px": [
            [float(point[0]), float(point[1])] for point in ordered
        ],
        "long_side_px": long_side,
        "short_side_px": short_side,
        "long_axis_yaw_deg": long_axis_yaw,
        "gripper_closing_yaw_deg": closing_yaw,
        "yaw_semantics": YAW_SEMANTICS,
    }


def _class_name(names: object, class_id: int) -> str:
    if isinstance(names, dict):
        value = names.get(class_id)
    elif isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        value = names[class_id]
    else:
        value = None
    if not isinstance(value, str):
        raise ValueError(f"model has no class name for id {class_id}")
    return value


def detect_image(
    model_path: Path,
    image_path: Path,
    confidence_threshold: float,
    device: str,
) -> tuple[np.ndarray, dict[str, Any]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"image decode failed: {image_path}")
    try:
        ultralytics = importlib.import_module("ultralytics")
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "Ultralytics is required for desktop can detection; install "
            "requirements-training.txt in the training environment"
        ) from error
    model = ultralytics.YOLO(str(model_path))
    results = model.predict(
        source=image,
        conf=confidence_threshold,
        device=device,
        verbose=False,
    )
    if len(results) != 1:
        raise RuntimeError(f"model returned {len(results)} image results")
    result = results[0]
    obb = result.obb
    if obb is None:
        raise ValueError("model result has no OBB output")
    corners = np.asarray(obb.xyxyxyxy.cpu().numpy(), dtype=np.float64)
    confidences = np.asarray(obb.conf.cpu().numpy(), dtype=np.float64)
    class_ids = np.asarray(obb.cls.cpu().numpy(), dtype=np.int64)
    candidates = []
    for box, confidence, class_id in zip(
        corners,
        confidences,
        class_ids,
        strict=True,
    ):
        if _class_name(result.names, int(class_id)) != "can":
            continue
        if float(confidence) < confidence_threshold:
            continue
        candidates.append(pose_from_corners(box, float(confidence)))
    if len(candidates) != 1:
        raise ValueError(
            "expected exactly one can above the confidence threshold, "
            f"detected {len(candidates)}"
        )
    return image, candidates[0]


def draw_detection(image: np.ndarray, detection: dict[str, Any]) -> np.ndarray:
    output = image.copy()
    corners = np.round(detection["corners_px"]).astype(np.int32)
    cv2.polylines(output, [corners], True, (0, 255, 0), 3, cv2.LINE_AA)
    center = tuple(np.round(detection["center_px"]).astype(int))
    cv2.circle(output, center, 5, (0, 0, 255), -1, cv2.LINE_AA)
    length = max(30.0, detection["long_side_px"] * 0.35)
    angle = math.radians(detection["long_axis_yaw_deg"])
    endpoint = (
        int(round(center[0] + length * math.cos(angle))),
        int(round(center[1] + length * math.sin(angle))),
    )
    cv2.line(output, center, endpoint, (255, 0, 0), 3, cv2.LINE_AA)
    label = (
        f"can {detection['confidence']:.2f} "
        f"axis={detection['long_axis_yaw_deg']:.1f}deg"
    )
    cv2.putText(
        output,
        label,
        (max(0, center[0] - 100), max(24, center[1] - 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        label,
        (max(0, center[0] - 100), max(24, center[1] - 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overlay", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if not 0.0 < args.confidence < 1.0:
            raise ValueError("--confidence must be within (0, 1)")
        if not args.model.is_file():
            raise ValueError(f"model is missing: {args.model}")
        if not args.image.is_file():
            raise ValueError(f"image is missing: {args.image}")
        image, detection = detect_image(
            args.model.resolve(),
            args.image.resolve(),
            args.confidence,
            args.device,
        )
        result = {
            "schema_version": 1,
            "status": STATUS,
            "motion_authorized": False,
            "robot_target_available": False,
            "image": str(args.image.resolve()),
            "model": str(args.model.resolve()),
            "detection": detection,
        }
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.output is not None:
            output = args.output.resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(serialized, encoding="utf-8")
        if args.overlay is not None:
            overlay = args.overlay.resolve()
            overlay.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(overlay), draw_detection(image, detection)):
                raise RuntimeError(f"failed to write overlay: {overlay}")
        print(serialized, end="")
        return 0
    except Exception as error:
        print(f"CAN_OBB_IMAGE_DETECTION_ERROR reason={error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
