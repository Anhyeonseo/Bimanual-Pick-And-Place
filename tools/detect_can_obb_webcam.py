#!/usr/bin/env python3
"""Preview live can OBB detections from a webcam using an ONNX runtime.

This is a read-only perception check. It opens no ROS entities and never
authorizes or commands robot motion. Press q or Esc in the preview window to
exit, or use Ctrl+C in the terminal.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws" / "src" / "so101_top_perception"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from detect_can_obb_image import draw_detection, pose_from_corners  # noqa: E402
from so101_top_perception.obb_detector import (  # noqa: E402
    decode_ultralytics_obb,
    letterbox,
)


WINDOW_NAME = "Can OBB webcam - q/Esc to quit"
STATUS = "CAN_OBB_WEBCAM_PREVIEW_PASS"


def parse_camera_source(value: str) -> int | str:
    """Treat a non-negative integer as an OpenCV camera index."""
    stripped = value.strip()
    if stripped.isdecimal():
        return int(stripped)
    if not stripped:
        raise ValueError("camera source must not be empty")
    return stripped


class CanOnnxBackend:
    """Use OpenCV DNN when supported, otherwise fall back to ONNX Runtime."""

    def __init__(self, model_path: Path, preference: str = "auto") -> None:
        self.name = ""
        self._net = None
        self._session = None
        if preference in ("auto", "opencv"):
            try:
                self._net = cv2.dnn.readNetFromONNX(str(model_path))
                self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                self.name = "opencv"
                return
            except cv2.error:
                if preference == "opencv":
                    raise
        try:
            import onnxruntime
        except ModuleNotFoundError as error:
            raise RuntimeError(
                "OpenCV cannot load this ONNX graph and onnxruntime is not "
                "installed; install onnxruntime or use OpenCV 4.10+"
            ) from error
        self._session = onnxruntime.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"],
        )
        self.name = "onnxruntime"

    def forward(self, blob: np.ndarray) -> np.ndarray:
        if self._net is not None:
            self._net.setInput(blob)
            return self._net.forward()
        assert self._session is not None
        input_name = self._session.get_inputs()[0].name
        return np.asarray(self._session.run(None, {input_name: blob})[0])


def infer_can_detections(
    backend: CanOnnxBackend,
    frame: np.ndarray,
    image_size: int,
    confidence: float,
    iou: float,
) -> list[dict]:
    """Run one raw Ultralytics OBB ONNX inference on a BGR frame."""
    model_input, transform = letterbox(frame, image_size, image_size, 114)
    blob = cv2.dnn.blobFromImage(
        model_input,
        scalefactor=1.0 / 255.0,
        size=(image_size, image_size),
        mean=(0.0, 0.0, 0.0),
        swapRB=True,
        crop=False,
    )
    output = backend.forward(blob)
    decoded = decode_ultralytics_obb(
        output,
        transform,
        class_count=1,
        pen_class_id=0,
        confidence_threshold=confidence,
        iou_threshold=iou,
        maximum_detections=5,
    )
    return [
        pose_from_corners(
            np.asarray(item["raw_corners_px"], dtype=np.float64),
            float(item["confidence"]),
        )
        for item in decoded
    ]


def draw_live_overlay(
    frame: np.ndarray,
    detections: list[dict],
    inference_ms: float,
) -> np.ndarray:
    output = frame.copy()
    for detection in detections:
        output = draw_detection(output, detection)
    if len(detections) == 0:
        state, color = "NO CAN", (0, 0, 220)
    elif len(detections) == 1:
        state, color = "CAN DETECTED", (0, 180, 0)
    else:
        state, color = f"MULTIPLE CANS: {len(detections)}", (0, 140, 255)
    cv2.rectangle(output, (0, 0), (output.shape[1], 32), color, -1)
    cv2.putText(
        output,
        f"{state}  inference={inference_ms:.1f}ms  q/Esc: quit",
        (8, 22),
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
    parser.add_argument(
        "--camera",
        default="0",
        help="OpenCV camera index or device path, for example 0 or /dev/video0",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--inference-hz", type=float, default=4.0)
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument(
        "--backend",
        choices=("auto", "opencv", "onnxruntime"),
        default="auto",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    capture = None
    try:
        model_path = args.model.resolve()
        if not model_path.is_file():
            raise ValueError(f"model is missing: {model_path}")
        if args.width <= 0 or args.height <= 0 or args.fps <= 0.0:
            raise ValueError("width, height and fps must be positive")
        if args.image_size <= 0 or args.image_size % 32:
            raise ValueError("image-size must be a positive multiple of 32")
        if not 0.0 < args.confidence < 1.0 or not 0.0 < args.iou < 1.0:
            raise ValueError("confidence and iou must be within (0, 1)")
        if not math.isfinite(args.inference_hz) or args.inference_hz <= 0.0:
            raise ValueError("inference-hz must be positive")

        source = parse_camera_source(args.camera)
        backend = CanOnnxBackend(model_path, args.backend)
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, args.fps)
        if not capture.isOpened():
            raise RuntimeError(f"failed to open camera: {args.camera}")

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        interval = 1.0 / args.inference_hz
        next_inference = 0.0
        detections: list[dict] = []
        inference_ms = 0.0
        print(
            f"CAN_OBB_WEBCAM_PREVIEW_READY camera={args.camera} "
            f"model={model_path} backend={backend.name} "
            "motion_authorized=false"
        )
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("webcam frame read failed")
            now = time.monotonic()
            if now >= next_inference:
                started = time.perf_counter()
                detections = infer_can_detections(
                    backend,
                    frame,
                    args.image_size,
                    args.confidence,
                    args.iou,
                )
                inference_ms = (time.perf_counter() - started) * 1000.0
                next_inference = now + interval
            cv2.imshow(
                WINDOW_NAME,
                draw_live_overlay(frame, detections, inference_ms),
            )
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
        print(f"{STATUS} motion_authorized=false")
        return 0
    except KeyboardInterrupt:
        print(f"{STATUS} interrupted=true motion_authorized=false")
        return 0
    except Exception as error:
        print(f"CAN_OBB_WEBCAM_PREVIEW_ERROR reason={error}", file=sys.stderr)
        return 2
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
