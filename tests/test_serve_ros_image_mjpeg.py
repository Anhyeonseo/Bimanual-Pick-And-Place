from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools/serve_ros_image_mjpeg.py").read_text(encoding="utf-8")


def test_ros_mjpeg_bridge_is_read_only_and_uses_sensor_qos() -> None:
    assert 'default="/perception/top/can_obb/debug"' in SOURCE
    assert "qos_profile_sensor_data" in SOURCE
    assert "create_subscription" in SOURCE
    assert "create_publisher" not in SOURCE
    assert "create_client" not in SOURCE
    assert "motion_authorized=false" in SOURCE


def test_ros_mjpeg_bridge_reports_first_frame() -> None:
    assert "ROS_IMAGE_MJPEG_READY" in SOURCE
    assert "ROS_IMAGE_MJPEG_FIRST_FRAME" in SOURCE
