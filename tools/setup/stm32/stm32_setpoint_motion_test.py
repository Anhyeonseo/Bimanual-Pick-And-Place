"""Low-amplitude six-axis binary setpoint motion/return verification."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import struct
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.lib.actuator_protocol import (  # noqa: E402
    Frame,
    MessageType,
    ProtocolError,
    StreamDecoder,
    decode_frame,
    encode_frame,
)
from tools.lib.joint_calibration import load_calibration, raw_to_urad  # noqa: E402


HELLO_PAYLOAD = struct.Struct("<BBBBIIII")
STATE_PAYLOAD = struct.Struct("<BBBBIIII")
ARM_RESPONSE_PAYLOAD = struct.Struct("<BB2xI")
SETPOINT_STATUS_PAYLOAD = struct.Struct("<BBBBIII")
AXIS_PATTERN = re.compile(r"AXIS ID=(\d+).*? POS=(\d+)")


def send_frame(
    port: object,
    message_type: MessageType,
    sequence: int,
    payload: bytes = b"",
    flags: int = 0,
) -> None:
    port.write(
        encode_frame(
            Frame(
                message_type=message_type,
                flags=flags,
                sequence=sequence,
                sender_time_ms=int(time.monotonic() * 1000) & 0xFFFFFFFF,
                payload=payload,
            )
        )
    )
    port.flush()


def read_frame(port: object, stage: str) -> Frame:
    packet = port.read_until(b"\x00")
    if not packet.endswith(b"\x00"):
        raise TimeoutError(f"{stage}: binary response timeout; raw={packet.hex()}")
    try:
        return decode_frame(packet)
    except ProtocolError as error:
        raise RuntimeError(
            f"{stage}: invalid binary response ({error}); raw={packet.hex()}"
        ) from error


def read_ascii_positions(port: object) -> list[int]:
    port.reset_input_buffer()
    port.write(b"S")
    port.flush()

    positions: dict[int, int] = {}
    lines: list[str] = []
    end_seen = False
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and not end_seen:
        raw_line = port.readline()
        if not raw_line:
            continue
        line = raw_line.decode("ascii", errors="replace").strip()
        lines.append(line)
        if line == "ALL_AXIS_STATUS_END":
            end_seen = True
        match = AXIS_PATTERN.search(line)
        if match:
            positions[int(match.group(1))] = int(match.group(2))

    if (not end_seen) or sorted(positions) != [1, 2, 3, 4, 5, 6]:
        raise RuntimeError(
            "ASCII preflight could not read all six positions; "
            f"received={lines!r}. Press NUCLEO RESET and verify 12 V servo power."
        )
    return [positions[joint_id] for joint_id in range(1, 7)]


def assert_start_pose(
    mode: str, positions: list[int], calibration: dict[str, object]
) -> None:
    tolerance = 30

    if mode == "home":
        unsafe = []
        for actual, joint in zip(positions, calibration["joints"]):
            if not (
                joint["minimum_raw"] - tolerance
                <= actual
                <= joint["maximum_raw"] + tolerance
            ):
                unsafe.append((joint["id"], actual))
        if unsafe:
            raise RuntimeError(
                f"home recovery start pose is outside the safe ranges: {unsafe}. "
                "No motion was commanded."
            )
        return

    expected = [2048, 2048, 2048, 2048, 2048, 2048]
    errors = [actual - target for actual, target in zip(positions, expected)]
    if any(abs(error) > tolerance for error in errors):
        raise RuntimeError(
            f"start pose rejected for mode={mode}: positions={positions}, "
            f"expected={expected}, tolerance={tolerance}. No motion was commanded."
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Move all six joints by about +3 degrees, or return them to zero."
    )
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=("out", "home"), required=True)
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required in .venv-host", file=sys.stderr)
        return 2

    calibration = load_calibration(REPO_ROOT / "config/single_arm_calibration.json")
    positive_urad = raw_to_urad(calibration, 0, 2082)
    target_urad = [positive_urad] * 6 if args.mode == "out" else [0] * 6

    try:
        with serial.Serial(args.port, args.baud, timeout=2.0) as port:
            positions = read_ascii_positions(port)
            assert_start_pose(args.mode, positions, calibration)
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
            hello_values = HELLO_PAYLOAD.unpack(hello.payload)
            calibration_hash = hello_values[5]

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
                raise RuntimeError(
                    f"ARM rejected: result={arm_result} state={arm_state}"
                )

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
            state_values = STATE_PAYLOAD.unpack(state.payload)
            first_apply_tick = (state_values[7] + 1500) & 0xFFFFFFFF

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
            completed = False
            maximum_error_raw = None
            next_heartbeat = time.monotonic()
            deadline = time.monotonic() + 5.0
            port.timeout = 0.02

            while time.monotonic() < deadline and not completed:
                now = time.monotonic()
                if now >= next_heartbeat:
                    send_frame(port, MessageType.HEARTBEAT, sequence)
                    sequence += 1
                    next_heartbeat = now + 0.1

                data = port.read(port.in_waiting or 1)
                for byte in data:
                    try:
                        response = decoder.push(byte)
                    except ProtocolError as error:
                        raise RuntimeError(f"motion response framing failed: {error}") from error
                    if response is None:
                        continue
                    if response.message_type is not MessageType.SETPOINT_STATUS:
                        continue
                    values = SETPOINT_STATUS_PAYLOAD.unpack(response.payload)
                    status, samples, state_code, detail = values[:4]
                    if response.sequence != motion_sequence:
                        continue
                    if status == 0:
                        accepted = True
                        print(
                            f"MOTION_ACCEPTED MODE={args.mode} "
                            f"SAMPLES={samples} STATE={state_code}"
                        )
                    elif status == 6:
                        completed = True
                        maximum_error_raw = detail
                    else:
                        raise RuntimeError(
                            f"motion failed: status={status} detail={detail}"
                        )

            if not accepted or not completed:
                raise TimeoutError(
                    f"motion completion timeout: accepted={accepted} completed={completed}"
                )

            port.timeout = 2.0
            disabled_ok = False
            for disable_attempt in range(3):
                send_frame(
                    port,
                    MessageType.DISABLE,
                    sequence + disable_attempt,
                )
                try:
                    disabled = read_frame(
                        port,
                        f"DISABLE_ATTEMPT_{disable_attempt + 1}",
                    )
                except TimeoutError:
                    time.sleep(0.1)
                    continue
                if (
                    disabled.message_type is MessageType.STATE_FEEDBACK
                    and STATE_PAYLOAD.unpack(disabled.payload)[1] == 0
                ):
                    disabled_ok = True
                    break
                time.sleep(0.1)
            if not disabled_ok:
                raise RuntimeError("DISABLE was not acknowledged after 3 attempts")

            print(
                f"BINARY_MOTION_OK MODE={args.mode} "
                f"TARGET_URAD={target_urad} MAX_ERROR_RAW={maximum_error_raw}"
            )
            return 0
    except Exception as error:
        print(f"BINARY_MOTION_FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
