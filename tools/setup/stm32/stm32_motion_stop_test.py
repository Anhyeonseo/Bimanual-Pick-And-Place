"""Verify binary SAFE_STOP during an active low-amplitude trajectory."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.actuator_protocol import (  # noqa: E402
    MessageType,
    ProtocolError,
    StreamDecoder,
)
from tools.lib.joint_calibration import load_calibration  # noqa: E402
from tools.setup.stm32.stm32_setpoint_motion_test import (  # noqa: E402
    ARM_RESPONSE_PAYLOAD,
    HELLO_PAYLOAD,
    SETPOINT_STATUS_PAYLOAD,
    STATE_PAYLOAD,
    assert_start_pose,
    read_ascii_positions,
    read_frame,
    send_frame,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stop a visible, low-amplitude six-axis binary motion."
    )
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--target-degrees", type=float, default=3.0)
    parser.add_argument("--stop-delay-ms", type=int, default=400)
    args = parser.parse_args()
    if not 1.0 <= args.target_degrees <= 8.0:
        parser.error("--target-degrees must be between 1 and 8")
    if not 200 <= args.stop_delay_ms <= 1000:
        parser.error("--stop-delay-ms must be between 200 and 1000")
    return args


def main() -> int:
    args = parse_args()
    try:
        import serial
    except ImportError:
        print("pyserial is required in .venv-host", file=sys.stderr)
        return 2

    calibration = load_calibration(REPO_ROOT / "config/single_arm_calibration.json")
    positive_urad = round(math.radians(args.target_degrees) * 1_000_000)
    target_urad = [positive_urad] * 6

    try:
        with serial.Serial(args.port, 115200, timeout=2.0) as port:
            positions = read_ascii_positions(port)
            assert_start_pose("out", positions, calibration)
            print(f"PREFLIGHT_POSITIONS={positions}")

            port.write(b"P")
            port.flush()
            ready = port.readline().decode("ascii", errors="replace").strip()
            if ready != "BINARY_PROTOCOL_READY_RESET_TO_EXIT":
                raise RuntimeError(f"binary mode acknowledgement missing: {ready!r}")

            sequence = 1
            send_frame(port, MessageType.HELLO_REQUEST, sequence)
            hello = read_frame(port, "HELLO")
            sequence += 1
            calibration_hash = HELLO_PAYLOAD.unpack(hello.payload)[5]

            send_frame(
                port,
                MessageType.ARM_REQUEST,
                sequence,
                struct.pack("<I", calibration_hash),
            )
            armed = read_frame(port, "ARM")
            sequence += 1
            arm_result, arm_state, returned_hash = ARM_RESPONSE_PAYLOAD.unpack(
                armed.payload
            )
            if arm_result != 0 or arm_state != 2 or returned_hash != calibration_hash:
                raise RuntimeError("ARM rejected")

            send_frame(port, MessageType.HEARTBEAT, sequence)
            sequence += 1
            send_frame(port, MessageType.ENABLE, sequence)
            enabled = read_frame(port, "ENABLE")
            sequence += 1
            if STATE_PAYLOAD.unpack(enabled.payload)[1] != 0:
                raise RuntimeError("ENABLE rejected")

            send_frame(port, MessageType.HEARTBEAT, sequence)
            sequence += 1
            send_frame(port, MessageType.GET_STATE, sequence)
            state = read_frame(port, "GET_STATE")
            sequence += 1
            first_apply_tick = (STATE_PAYLOAD.unpack(state.payload)[7] + 1500) & 0xFFFFFFFF

            payload = struct.pack(
                "<IBBH" + "I" + "i" * 12,
                first_apply_tick,
                1,
                1,
                0,
                0,
                *(target_urad + [0] * 6),
            )
            motion_sequence = sequence
            send_frame(
                port,
                MessageType.SETPOINT_BATCH,
                motion_sequence,
                payload,
            )
            sequence += 1

            decoder = StreamDecoder()
            accepted = False
            stop_sent = False
            stop_acknowledged = False
            motion_stopped = False
            stop_sequence = None
            stop_at = None
            next_heartbeat = time.monotonic()
            deadline = time.monotonic() + 5.0
            port.timeout = 0.02

            while time.monotonic() < deadline and not (
                stop_acknowledged and motion_stopped
            ):
                now = time.monotonic()
                if now >= next_heartbeat:
                    send_frame(port, MessageType.HEARTBEAT, sequence)
                    sequence += 1
                    next_heartbeat = now + 0.1

                if accepted and not stop_sent and stop_at is not None and now >= stop_at:
                    stop_sequence = sequence
                    send_frame(port, MessageType.SAFE_STOP, stop_sequence)
                    sequence += 1
                    stop_sent = True
                    print("SAFE_STOP_SENT_DURING_MOTION")

                data = port.read(port.in_waiting or 1)
                for byte in data:
                    try:
                        response = decoder.push(byte)
                    except ProtocolError as error:
                        raise RuntimeError(f"stop response framing failed: {error}") from error
                    if response is None:
                        continue

                    if response.message_type is MessageType.SETPOINT_STATUS:
                        status, _, _, detail, _, _, _ = SETPOINT_STATUS_PAYLOAD.unpack(
                            response.payload
                        )
                        if response.sequence != motion_sequence:
                            continue
                        if status == 0:
                            accepted = True
                            stop_at = (
                                time.monotonic() + args.stop_delay_ms / 1000.0
                            )
                            print(
                                "MOTION_ACCEPTED_STOP_SCHEDULED_"
                                f"{args.stop_delay_ms}MS "
                                f"TARGET_DEG={args.target_degrees:g}"
                            )
                        elif status == 8:
                            motion_stopped = True
                        elif status == 6:
                            raise RuntimeError("motion completed before SAFE_STOP took effect")
                        else:
                            raise RuntimeError(
                                f"motion failed before stop: status={status} detail={detail}"
                            )
                    elif (
                        response.message_type is MessageType.STATE_FEEDBACK
                        and stop_sequence is not None
                        and response.sequence == stop_sequence
                    ):
                        stop_values = STATE_PAYLOAD.unpack(response.payload)
                        if stop_values[0] == 1 and stop_values[1] == 0:
                            stop_acknowledged = True

            # SETPOINT_STATUS=8 proves that the executor observed the stop
            # latch.  If the command's immediate STATE_FEEDBACK frame was
            # lost, independently query the latched state before declaring
            # failure.  This keeps the safety assertion strict while making
            # the serial test tolerant of one missing response frame.
            if accepted and motion_stopped and not stop_acknowledged:
                state_sequence = sequence
                sequence += 1
                port.timeout = 2.0
                send_frame(port, MessageType.GET_STATE, state_sequence)
                state_response = read_frame(port, "SAFE_STOP_GET_STATE")
                if (
                    state_response.message_type is MessageType.STATE_FEEDBACK
                    and state_response.sequence == state_sequence
                ):
                    stop_values = STATE_PAYLOAD.unpack(state_response.payload)
                    if stop_values[0] == 1:
                        stop_acknowledged = True
                        print("SAFE_STOP_LATCH_CONFIRMED_BY_GET_STATE")

            if not accepted or not stop_acknowledged or not motion_stopped:
                raise TimeoutError(
                    "SAFE_STOP verification timeout: "
                    f"accepted={accepted} ack={stop_acknowledged} stopped={motion_stopped}"
                )

            send_frame(port, MessageType.HEARTBEAT, sequence)
            sequence += 1
            port.timeout = 2.0
            cleared = False
            for attempt in range(3):
                clear_sequence = sequence
                sequence += 1
                send_frame(port, MessageType.CLEAR_FAULT, clear_sequence)
                clear_response = read_frame(port, f"CLEAR_ATTEMPT_{attempt + 1}")
                clear_values = STATE_PAYLOAD.unpack(clear_response.payload)
                if clear_values[0] == 0 and clear_values[1] == 0:
                    cleared = True
                    break
                if clear_values[1] != 2:
                    break
                time.sleep(0.1)
            if not cleared:
                raise RuntimeError("SAFE_STOP latch did not clear after safety checks")

            print("BINARY_MOTION_SAFE_STOP_OK")
            print("RESET_THEN_RUN_HOME_RECOVERY")
            return 0
    except Exception as error:
        print(f"BINARY_MOTION_SAFE_STOP_FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
