#!/usr/bin/env python3
"""Pure helpers for versioned tabletop object-pose capture datasets."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any


SCHEMA_VERSION = 1
DATASET_KIND = "tabletop_object_pose_capture"
CAPTURE_STATUS = "READ_ONLY_OBJECT_POSE_CAPTURE_PASS"
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,79}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(document: dict[str, Any]) -> str:
    encoded = json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def validate_identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER_PATTERN.fullmatch(value):
        raise ValueError(
            f"{field} must match {IDENTIFIER_PATTERN.pattern}: {value!r}"
        )
    return value


def _positive_number(value: object, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} must be a number") from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return number


def validate_capture_config(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
    if document.get("dataset_kind") != DATASET_KIND:
        raise ValueError(f"dataset_kind must be {DATASET_KIND}")
    validate_identifier(document.get("dataset_id"), "dataset_id")
    if document.get("motion_authorized") is not False:
        raise ValueError("capture config must keep motion_authorized=false")
    if document.get("robot_target_available") is not False:
        raise ValueError(
            "capture config must keep robot_target_available=false"
        )

    task = document.get("task")
    if not isinstance(task, dict):
        raise ValueError("task config is required")
    if task.get("name") != "dual_arm_can_disposal":
        raise ValueError("task.name must be dual_arm_can_disposal")
    if task.get("initial_state") != "lying":
        raise ValueError("task.initial_state must be lying")
    if task.get("goal_state") != "disposed":
        raise ValueError("task.goal_state must be disposed")
    routing = task.get("routing")
    if not isinstance(routing, dict):
        raise ValueError("task.routing is required")
    if routing.get("right_workspace") != "right_arm_to_right_bin":
        raise ValueError("task.routing.right_workspace is invalid")
    if routing.get("left_workspace") != (
        "left_arm_to_fixed_handoff_then_right_bin"
    ):
        raise ValueError("task.routing.left_workspace is invalid")

    object_config = document.get("object")
    if not isinstance(object_config, dict):
        raise ValueError("object config is required")
    validate_identifier(object_config.get("class_name"), "object.class_name")
    validate_identifier(object_config.get("instance_id"), "object.instance_id")
    states = object_config.get("allowed_states")
    if (
        not isinstance(states, list)
        or not states
        or any(not isinstance(value, str) or not value for value in states)
        or len(set(states)) != len(states)
    ):
        raise ValueError("object.allowed_states must contain unique strings")
    dimensions = object_config.get("dimensions_m")
    if not isinstance(dimensions, dict):
        raise ValueError("object.dimensions_m is required")
    for name, value in dimensions.items():
        if value is not None:
            _positive_number(value, f"object.dimensions_m.{name}")
    if object_config.get("mass_kg") is not None:
        _positive_number(object_config["mass_kg"], "object.mass_kg")

    camera = document.get("camera")
    if not isinstance(camera, dict):
        raise ValueError("camera config is required")
    topic = camera.get("image_topic")
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise ValueError("camera.image_topic must be an absolute ROS topic")
    frames = camera.get("frames_per_capture")
    settle_frames = camera.get("settle_frames")
    if not isinstance(frames, int) or frames < 1:
        raise ValueError("camera.frames_per_capture must be a positive integer")
    if not isinstance(settle_frames, int) or settle_frames < 0:
        raise ValueError("camera.settle_frames must be a nonnegative integer")
    _positive_number(camera.get("interval_s"), "camera.interval_s")
    _positive_number(camera.get("timeout_s"), "camera.timeout_s")
    minimum_sharpness = float(camera.get("minimum_sharpness", 0.0))
    if not math.isfinite(minimum_sharpness) or minimum_sharpness < 0.0:
        raise ValueError("camera.minimum_sharpness must be finite and nonnegative")
    if camera.get("image_format") != "png":
        raise ValueError("camera.image_format must be png")

    annotation = document.get("annotation")
    if not isinstance(annotation, dict):
        raise ValueError("annotation config is required")
    frame_id = annotation.get("ground_truth_frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError("annotation.ground_truth_frame_id is required")
    if annotation.get("yaw_semantics") != "undirected_long_axis_modulo_pi":
        raise ValueError(
            "annotation.yaw_semantics must be "
            "undirected_long_axis_modulo_pi"
        )
    return document


def load_capture_config(path: Path) -> dict[str, Any]:
    return validate_capture_config(load_json(path))


def normalize_undirected_yaw_deg(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("ground-truth yaw must be finite")
    normalized = value % 180.0
    return 0.0 if math.isclose(normalized, 180.0) else normalized


def build_annotation(
    *,
    frame_id: str,
    x_m: float | None,
    y_m: float | None,
    yaw_deg: float | None,
) -> dict[str, Any]:
    if (x_m is None) != (y_m is None):
        raise ValueError("ground-truth x and y must be supplied together")
    for name, value in (("x_m", x_m), ("y_m", y_m)):
        if value is not None and not math.isfinite(value):
            raise ValueError(f"ground-truth {name} must be finite")
    normalized_yaw = (
        None if yaw_deg is None else normalize_undirected_yaw_deg(yaw_deg)
    )
    measured_fields = sum(
        value is not None for value in (x_m, y_m, normalized_yaw)
    )
    if measured_fields == 0:
        status = "pending"
    elif measured_fields == 3:
        status = "measured"
    else:
        status = "partially_measured"
    return {
        "status": status,
        "frame_id": frame_id,
        "center_m": None if x_m is None else [float(x_m), float(y_m)],
        "long_axis_yaw_deg": normalized_yaw,
        "yaw_semantics": "undirected_long_axis_modulo_pi",
    }


def make_capture_document(
    *,
    config: dict[str, Any],
    capture_id: str,
    object_state: str,
    position_label: str,
    annotation: dict[str, Any],
    conditions: dict[str, str],
    frames: list[dict[str, Any]],
    notes: str,
) -> dict[str, Any]:
    validate_identifier(capture_id, "capture_id")
    allowed_states = config["object"]["allowed_states"]
    if object_state not in allowed_states:
        raise ValueError(
            f"object state must be one of {allowed_states}: {object_state!r}"
        )
    if not position_label.strip():
        raise ValueError("position_label must not be empty")
    if not frames:
        raise ValueError("capture must contain at least one frame")
    for field in ("background", "lighting", "glare"):
        if not conditions.get(field, "").strip():
            raise ValueError(f"conditions.{field} must not be empty")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": CAPTURE_STATUS,
        "captured_at_utc": utc_now(),
        "motion_authorized": False,
        "robot_target_available": False,
        "capture_id": capture_id,
        "dataset_id": config["dataset_id"],
        "object": {
            "class_name": config["object"]["class_name"],
            "instance_id": config["object"]["instance_id"],
            "state": object_state,
            "position_label": position_label.strip(),
            "dimensions_m": config["object"]["dimensions_m"],
        },
        "annotation": annotation,
        "conditions": {
            field: conditions[field].strip()
            for field in ("background", "lighting", "glare")
        },
        "camera": {
            "image_topic": config["camera"]["image_topic"],
            "image_format": "png",
        },
        "frames": frames,
        "notes": notes.strip(),
    }


def update_dataset_manifest(
    dataset_root: Path,
    config: dict[str, Any],
    config_sha256: str,
    capture_document: dict[str, Any],
) -> Path:
    dataset_root = dataset_root.resolve()
    manifest_path = dataset_root / "dataset.json"
    capture_id = capture_document["capture_id"]
    capture_path = dataset_root / "captures" / capture_id / "capture.json"
    if not capture_path.is_file():
        raise ValueError(f"capture metadata is missing: {capture_path}")
    relative_capture = capture_path.relative_to(dataset_root).as_posix()
    entry = {
        "id": capture_id,
        "path": relative_capture,
        "sha256": file_sha256(capture_path),
        "object_state": capture_document["object"]["state"],
        "position_label": capture_document["object"]["position_label"],
    }

    if manifest_path.exists():
        manifest = load_json(manifest_path)
        if manifest.get("dataset_id") != config["dataset_id"]:
            raise ValueError("existing manifest dataset_id does not match config")
        if manifest.get("capture_config_sha256") != config_sha256:
            raise ValueError(
                "capture config changed; start a new dataset generation"
            )
        captures = manifest.get("captures")
        if not isinstance(captures, list):
            raise ValueError("existing manifest captures must be a list")
        if any(item.get("id") == capture_id for item in captures):
            raise ValueError(f"duplicate capture id: {capture_id}")
        captures.append(entry)
        manifest["updated_at_utc"] = utc_now()
    else:
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "dataset_kind": DATASET_KIND,
            "dataset_id": config["dataset_id"],
            "created_at_utc": utc_now(),
            "updated_at_utc": utc_now(),
            "motion_authorized": False,
            "robot_target_available": False,
            "capture_config_sha256": config_sha256,
            "object": config["object"],
            "annotation": config["annotation"],
            "captures": [entry],
        }
    atomic_write_json(manifest_path, manifest)
    return manifest_path
