from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from auto_label_can_obb import detect_red_can_obb, yolo_obb_line  # noqa: E402


def test_detect_red_can_obb_fits_a_rotated_red_can():
    image = np.full((240, 320, 3), 240, dtype=np.uint8)
    expected = cv2.boxPoints(((160, 120), (120, 50), 30))
    cv2.fillConvexPoly(image, np.round(expected).astype(np.int32), (20, 20, 190))

    corners, metrics = detect_red_can_obb(image)

    center = corners.mean(axis=0)
    assert center == pytest.approx([160, 120], abs=2)
    assert metrics["box_aspect_ratio"] == pytest.approx(2.4, rel=0.15)
    assert metrics["review_required"] is False
    values = yolo_obb_line(corners, 320, 240).split()
    assert values[0] == "0"
    assert len(values) == 9
    assert all(0.0 <= float(value) <= 1.0 for value in values[1:])


def test_detect_red_can_obb_rejects_an_empty_background():
    image = np.full((240, 320, 3), 240, dtype=np.uint8)

    with pytest.raises(ValueError, match="no red can candidate"):
        detect_red_can_obb(image)


def test_detect_red_can_obb_accepts_a_close_can_over_190_pixels_long():
    image = np.full((480, 640, 3), 240, dtype=np.uint8)
    expected = cv2.boxPoints(((150, 400), (220, 85), 0))
    cv2.fillConvexPoly(image, np.round(expected).astype(np.int32), (20, 20, 190))

    corners, metrics = detect_red_can_obb(image)

    assert corners.mean(axis=0) == pytest.approx([150, 400], abs=2)
    assert metrics["box_aspect_ratio"] == pytest.approx(220 / 85, rel=0.15)
