#!/usr/bin/env python3
"""Capture a read-only tabletop object-pose sample from a ROS image topic.

The tool only subscribes to an image topic. It never creates a publisher,
Action client, service client, serial connection, or robot motion command.
Each invocation records one object placement as multiple lossless PNG frames.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from object_pose_dataset import (
    atomic_write_json,
    build_annotation,
    canonical_sha256,
    file_sha256,
    load_capture_config,
    make_capture_document,
    update_dataset_manifest,
    validate_identifier,
)


def decode_image(message: Image) -> np.ndarray:
    if message.width <= 0 or message.height <= 0:
        raise ValueError("image dimensions must be positive")
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "mono8": 1}
    channels = channels_by_encoding.get(message.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    required_step = int(message.width) * channels
    if int(message.step) < required_step:
        raise ValueError("image step is shorter than one packed row")
    required_size = int(message.step) * int(message.height)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < required_size:
        raise ValueError("image data is truncated")
    rows = raw[:required_size].reshape(int(message.height), int(message.step))
    packed = rows[:, :required_step]
    if channels == 1:
        gray = packed.reshape(int(message.height), int(message.width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    image = packed.reshape(int(message.height), int(message.width), channels)
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


class ObjectPoseCapture(Node):
    def __init__(self, args: argparse.Namespace, config: dict) -> None:
        super().__init__("object_pose_dataset_capture")
        self._args = args
        self._config = config
        self._received = 0
        self._accepted: list[tuple[np.ndarray, dict]] = []
        self._target_frame_count = (
            args.frames_per_capture
            if args.frames_per_capture is not None
            else config["camera"]["frames_per_capture"]
        )
        self._last_accept_monotonic = -math.inf
        self.failure: str | None = None
        self.finished = False
        self._subscription = self.create_subscription(
            Image,
            config["camera"]["image_topic"],
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: Image) -> None:
        if self.finished or self.failure is not None:
            return
        self._received += 1
        if self._received <= self._config["camera"]["settle_frames"]:
            return
        now = time.monotonic()
        if now - self._last_accept_monotonic < self._config["camera"]["interval_s"]:
            return
        try:
            image = decode_image(message)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            if sharpness < self._config["camera"]["minimum_sharpness"]:
                self.get_logger().warning(
                    "OBJECT_POSE_FRAME_REJECTED "
                    f"sharpness={sharpness:.3f}"
                )
                return
            record = {
                "source_stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                "source_frame_id": str(message.header.frame_id),
                "width": int(message.width),
                "height": int(message.height),
                "source_encoding": str(message.encoding),
                "sharpness": sharpness,
            }
            self._accepted.append((image, record))
            self._last_accept_monotonic = now
            self.get_logger().info(
                "OBJECT_POSE_FRAME_ACCEPTED "
                f"count={len(self._accepted)}/"
                f"{self._target_frame_count} "
                f"sharpness={sharpness:.3f}"
            )
            if len(self._accepted) >= self._target_frame_count:
                self.finished = True
        except Exception as error:
            self.failure = str(error)

    @property
    def accepted(self) -> list[tuple[np.ndarray, dict]]:
        return list(self._accepted)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--state", required=True)
    parser.add_argument("--position-label", required=True)
    parser.add_argument(
        "--frames-per-capture",
        type=int,
        help="override the configured frame count for this capture",
    )
    parser.add_argument("--ground-truth-x-m", type=float)
    parser.add_argument("--ground-truth-y-m", type=float)
    parser.add_argument("--ground-truth-yaw-deg", type=float)
    parser.add_argument("--background", default="default")
    parser.add_argument("--lighting", default="default")
    parser.add_argument("--glare", default="none")
    parser.add_argument("--notes", default="")
    return parser.parse_args()


def write_capture(
    args: argparse.Namespace,
    config: dict,
    accepted: list[tuple[np.ndarray, dict]],
) -> tuple[Path, Path]:
    dataset_root = args.dataset_root.resolve()
    capture_path = dataset_root / f"{args.capture_id}.json"
    existing_frames = list(dataset_root.glob(f"{args.capture_id}_frame_*.png"))
    if capture_path.exists() or existing_frames:
        raise ValueError(f"capture already exists: {args.capture_id}")
    dataset_root.mkdir(parents=True, exist_ok=True)
    created_paths: list[Path] = []
    try:
        frame_records = []
        for index, (image, record) in enumerate(accepted):
            image_path = dataset_root / (
                f"{args.capture_id}_frame_{index:03d}.png"
            )
            if not cv2.imwrite(str(image_path), image):
                raise RuntimeError(f"failed to write {image_path}")
            created_paths.append(image_path)
            frame_records.append(
                {
                    **record,
                    "file": image_path.name,
                    "sha256": file_sha256(image_path),
                }
            )
        annotation = build_annotation(
            frame_id=config["annotation"]["ground_truth_frame_id"],
            x_m=args.ground_truth_x_m,
            y_m=args.ground_truth_y_m,
            yaw_deg=args.ground_truth_yaw_deg,
        )
        document = make_capture_document(
            config=config,
            capture_id=args.capture_id,
            object_state=args.state,
            position_label=args.position_label,
            annotation=annotation,
            conditions={
                "background": args.background,
                "lighting": args.lighting,
                "glare": args.glare,
            },
            frames=frame_records,
            notes=args.notes,
        )
        atomic_write_json(capture_path, document)
        created_paths.append(capture_path)
        manifest_path = update_dataset_manifest(
            dataset_root,
            config,
            canonical_sha256(config),
            document,
        )
        return capture_path, manifest_path
    except Exception:
        for path in reversed(created_paths):
            path.unlink(missing_ok=True)
        raise


def main() -> int:
    args = parse_args()
    validate_identifier(args.capture_id, "capture_id")
    config = load_capture_config(args.config.resolve())
    if args.frames_per_capture is not None and args.frames_per_capture <= 0:
        raise ValueError("--frames-per-capture must be positive")
    if args.state not in config["object"]["allowed_states"]:
        raise ValueError(
            f"--state must be one of {config['object']['allowed_states']}"
        )
    if (args.ground_truth_x_m is None) != (args.ground_truth_y_m is None):
        raise ValueError("ground-truth x and y must be supplied together")
    dataset_root = args.dataset_root.resolve()
    capture_path = dataset_root / f"{args.capture_id}.json"
    existing_frames = list(dataset_root.glob(f"{args.capture_id}_frame_*.png"))
    if capture_path.exists() or existing_frames:
        raise ValueError(f"capture already exists: {args.capture_id}")

    rclpy.init()
    node = ObjectPoseCapture(args, config)
    deadline = time.monotonic() + config["camera"]["timeout_s"]
    try:
        while rclpy.ok() and not node.finished and node.failure is None:
            if time.monotonic() >= deadline:
                node.failure = "capture timed out before enough frames arrived"
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.failure = "capture interrupted"
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if node.failure is not None:
        print(f"OBJECT_POSE_CAPTURE_FAIL reason={node.failure}")
        return 2

    capture_path, manifest_path = write_capture(
        args,
        config,
        node.accepted,
    )
    print(
        "OBJECT_POSE_CAPTURE_PASS "
        f"capture={capture_path} manifest={manifest_path} "
        f"frames={len(node.accepted)} motion_authorized=false"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
