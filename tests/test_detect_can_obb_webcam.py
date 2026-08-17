from pathlib import Path
import importlib.util

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "detect_can_obb_webcam.py"
SPEC = importlib.util.spec_from_file_location("detect_can_obb_webcam", MODULE_PATH)
module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(module)


def test_parse_camera_source_accepts_index_and_device_path():
    assert module.parse_camera_source("0") == 0
    assert module.parse_camera_source(" 2 ") == 2
    assert module.parse_camera_source("/dev/video4") == "/dev/video4"


def test_parse_camera_source_rejects_empty_value():
    with pytest.raises(ValueError, match="must not be empty"):
        module.parse_camera_source("  ")


def test_draw_live_overlay_preserves_frame_shape():
    frame = np.zeros((120, 160, 3), dtype=np.uint8)

    output = module.draw_live_overlay(frame, [], 12.5)

    assert output.shape == frame.shape
    assert np.any(output != frame)


def test_headless_preview_contract_avoids_highgui():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert 'parser.add_argument("--mjpeg-port", type=int, default=8090)' in source
    assert "if args.headless:" in source
    assert 'ThreadingHTTPServer(("0.0.0.0", port), Handler)' in source
    assert "pen_class_id=" not in source
    assert "target_class_id=" not in source
