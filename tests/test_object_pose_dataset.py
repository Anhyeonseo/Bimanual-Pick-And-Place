import argparse
import ast
import json
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"

import sys

if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from object_pose_dataset import (  # noqa: E402
    atomic_write_json,
    build_annotation,
    canonical_sha256,
    load_capture_config,
    make_capture_document,
    normalize_undirected_yaw_deg,
    update_dataset_manifest,
)
from capture_object_pose_dataset import write_capture  # noqa: E402


CONFIG_PATH = ROOT / "config" / "can_disposal_capture.json"


def frame(name: str = "frame_000.png") -> dict:
    return {
        "file": name,
        "sha256": "a" * 64,
        "source_stamp": {"sec": 1, "nanosec": 2},
        "source_frame_id": "top_camera_optical_frame",
        "width": 640,
        "height": 480,
        "source_encoding": "rgb8",
        "sharpness": 100.0,
    }


def capture(config: dict, capture_id: str) -> dict:
    return make_capture_document(
        config=config,
        capture_id=capture_id,
        object_state="lying",
        position_label="center",
        annotation=build_annotation(
            frame_id="top_board",
            x_m=0.1,
            y_m=-0.2,
            yaw_deg=225.0,
        ),
        conditions={
            "background": "table",
            "lighting": "room",
            "glare": "low",
        },
        frames=[frame()],
        notes="",
    )


def write_capture_metadata(root: Path, document: dict) -> None:
    path = root / f"{document['capture_id']}.json"
    atomic_write_json(path, document)


def test_can_capture_config_is_a_read_only_generic_dataset_contract():
    config = load_capture_config(CONFIG_PATH)

    assert config["object"]["class_name"] == "can"
    assert config["motion_authorized"] is False
    assert config["robot_target_available"] is False
    assert config["camera"]["frames_per_capture"] == 5
    assert config["task"]["goal_state"] == "disposed"
    assert config["task"]["routing"]["right_workspace"] == (
        "right_arm_to_right_bin"
    )


@pytest.mark.parametrize(
    ("given", "expected"),
    [(0.0, 0.0), (45.0, 45.0), (180.0, 0.0), (225.0, 45.0), (-45.0, 135.0)],
)
def test_undirected_yaw_is_normalized_modulo_180(given, expected):
    assert normalize_undirected_yaw_deg(given) == pytest.approx(expected)


def test_annotation_allows_capture_before_manual_measurement():
    annotation = build_annotation(
        frame_id="top_board",
        x_m=None,
        y_m=None,
        yaw_deg=None,
    )

    assert annotation["status"] == "pending"
    assert annotation["center_m"] is None
    assert annotation["long_axis_yaw_deg"] is None


def test_annotation_marks_complete_manual_pose_as_measured():
    annotation = build_annotation(
        frame_id="top_board",
        x_m=0.1,
        y_m=-0.2,
        yaw_deg=225.0,
    )

    assert annotation["status"] == "measured"
    assert annotation["long_axis_yaw_deg"] == pytest.approx(45.0)


def test_annotation_rejects_only_one_position_axis():
    with pytest.raises(ValueError, match="supplied together"):
        build_annotation(
            frame_id="top_board",
            x_m=0.1,
            y_m=None,
            yaw_deg=0.0,
        )


def test_manifest_is_created_and_extended_without_overwriting(tmp_path):
    config = load_capture_config(CONFIG_PATH)
    config_hash = canonical_sha256(config)
    first = capture(config, "center_yaw000_trial01")
    second = capture(config, "left_yaw045_trial01")
    write_capture_metadata(tmp_path, first)
    update_dataset_manifest(tmp_path, config, config_hash, first)
    write_capture_metadata(tmp_path, second)
    manifest_path = update_dataset_manifest(
        tmp_path, config, config_hash, second
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert [entry["id"] for entry in manifest["captures"]] == [
        "center_yaw000_trial01",
        "left_yaw045_trial01",
    ]
    assert manifest["motion_authorized"] is False


def test_manifest_rejects_duplicate_capture_id(tmp_path):
    config = load_capture_config(CONFIG_PATH)
    config_hash = canonical_sha256(config)
    document = capture(config, "center_yaw000_trial01")
    write_capture_metadata(tmp_path, document)
    update_dataset_manifest(tmp_path, config, config_hash, document)

    with pytest.raises(ValueError, match="duplicate capture id"):
        update_dataset_manifest(tmp_path, config, config_hash, document)


def test_manifest_rejects_config_drift(tmp_path):
    config = load_capture_config(CONFIG_PATH)
    document = capture(config, "center_yaw000_trial01")
    write_capture_metadata(tmp_path, document)
    update_dataset_manifest(
        tmp_path,
        config,
        canonical_sha256(config),
        document,
    )
    changed = json.loads(json.dumps(config))
    changed["camera"]["frames_per_capture"] = 6
    second = capture(changed, "center_yaw045_trial01")
    write_capture_metadata(tmp_path, second)

    with pytest.raises(ValueError, match="config changed"):
        update_dataset_manifest(
            tmp_path,
            changed,
            canonical_sha256(changed),
            second,
        )


def test_write_capture_creates_lossless_frames_and_manifest(tmp_path):
    config = load_capture_config(CONFIG_PATH)
    arguments = argparse.Namespace(
        dataset_root=tmp_path,
        capture_id="center_yaw045_trial01",
        state="lying",
        position_label="center",
        ground_truth_x_m=0.1,
        ground_truth_y_m=-0.2,
        ground_truth_yaw_deg=45.0,
        background="table",
        lighting="room",
        glare="low",
        notes="test capture",
    )
    image = np.zeros((8, 12, 3), dtype=np.uint8)
    record = {
        "source_stamp": {"sec": 1, "nanosec": 2},
        "source_frame_id": "top_camera_optical_frame",
        "width": 12,
        "height": 8,
        "source_encoding": "rgb8",
        "sharpness": 100.0,
    }

    capture_path, manifest_path = write_capture(
        arguments,
        config,
        [(image, record)],
    )

    capture_document = json.loads(capture_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert (
        capture_path.parent
        / f"{arguments.capture_id}_frame_000.png"
    ).is_file()
    assert not (tmp_path / "captures").exists()
    assert capture_document["annotation"]["status"] == "measured"
    assert manifest["captures"][0]["id"] == arguments.capture_id


def test_capture_tool_has_no_robot_command_ros_api():
    source = (TOOLS / "capture_object_pose_dataset.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    attributes = {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }

    assert "create_publisher" not in attributes
    assert "create_client" not in attributes
    assert "create_service" not in attributes
