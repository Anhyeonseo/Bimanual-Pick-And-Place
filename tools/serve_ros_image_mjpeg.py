#!/usr/bin/env python3
"""Serve one ROS Image topic as a read-only MJPEG browser preview."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from capture_object_pose_dataset import decode_image  # noqa: E402
from detect_can_obb_webcam import MjpegPreview  # noqa: E402


class RosImageMjpegBridge(Node):
    def __init__(self, topic: str, port: int) -> None:
        super().__init__("ros_image_mjpeg_bridge")
        self._preview = MjpegPreview(port)
        self._frame_count = 0
        self._subscription = self.create_subscription(
            Image,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"ROS_IMAGE_MJPEG_READY topic={topic} port={port} "
            "motion_authorized=false"
        )

    def _on_image(self, message: Image) -> None:
        self._preview.publish(decode_image(message))
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info("ROS_IMAGE_MJPEG_FIRST_FRAME")

    def close(self) -> None:
        self._preview.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/perception/top/can_obb/debug",
    )
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()
    if not args.topic.startswith("/"):
        parser.error("topic must be absolute")
    if not 1 <= args.port <= 65535:
        parser.error("port must be within 1..65535")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RosImageMjpegBridge(args.topic, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
