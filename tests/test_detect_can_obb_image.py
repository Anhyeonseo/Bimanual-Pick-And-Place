import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "detect_can_obb_image.py"
SPEC = importlib.util.spec_from_file_location("detect_can_obb_image", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(
    ("given", "expected"),
    [(0.0, 0.0), (45.0, 45.0), (180.0, 0.0), (-30.0, 150.0)],
)
def test_axis_yaw_is_undirected(given, expected):
    assert MODULE.normalize_axis_yaw_deg(given) == pytest.approx(expected)


@pytest.mark.parametrize("angle_deg", [0.0, 30.0, 75.0, 120.0, 165.0])
def test_pose_from_corners_extracts_center_axis_and_crosswise_grip(angle_deg):
    rectangle = ((320.0, 240.0), (150.0, 60.0), angle_deg)
    corners = cv2.boxPoints(rectangle)

    pose = MODULE.pose_from_corners(corners, confidence=0.9)

    assert pose["center_px"] == pytest.approx([320.0, 240.0], abs=1e-4)
    assert pose["long_side_px"] == pytest.approx(150.0, abs=1e-3)
    assert pose["short_side_px"] == pytest.approx(60.0, abs=1e-3)
    assert pose["long_axis_yaw_deg"] == pytest.approx(
        angle_deg % 180.0,
        abs=1e-3,
    )
    expected_closing = (angle_deg + 90.0) % 180.0
    assert pose["gripper_closing_yaw_deg"] == pytest.approx(
        expected_closing,
        abs=1e-3,
    )


def test_pose_rejects_degenerate_box():
    corners = np.array(
        [[1.0, 1.0], [2.0, 1.0], [2.0, 1.0], [1.0, 1.0]],
        dtype=np.float64,
    )

    with pytest.raises(ValueError, match="area is too small"):
        MODULE.pose_from_corners(corners, confidence=0.9)


def test_draw_detection_preserves_image_shape():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    corners = cv2.boxPoints(((320.0, 240.0), (150.0, 60.0), 30.0))
    pose = MODULE.pose_from_corners(corners, confidence=0.9)

    output = MODULE.draw_detection(image, pose)

    assert output.shape == image.shape
    assert np.any(output != image)
