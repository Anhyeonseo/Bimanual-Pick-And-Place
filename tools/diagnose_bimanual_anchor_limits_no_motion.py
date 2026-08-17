#!/usr/bin/env python3
"""Diagnose the first F8.9 automatic anchor limit failure without motion."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import serial

from single_arm_bridge.bimanual_stream_adapter import (
    CALIBRATION_HASH,
    CANONICAL_JOINT_NAMES,
    F8_CAPABILITIES,
    F8_FIRMWARE_VERSION,
)
from single_arm_bridge.device_discovery import resolve_serial_device
from single_arm_bridge.serial_port import open_exclusive_serial
from single_arm_bridge.stream_transport_v2 import StreamValidationTransportV2


CONFIRMATION = "DIAGNOSE_BIMANUAL_ANCHOR_LIMITS_NO_MOTION"
HOST_BAUD = 921_600
RAW_MODULUS = 4096
EXPECTED_PROTOCOL_VERSION = 2
EXPECTED_JOINT_COUNT = 12


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--confirmation", required=True)
    parser.add_argument(
        "--limits",
        type=Path,
        default=root / "config/bimanual_operational_limits.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "artifacts/can_to_bin/anchor_limit_diagnostic.json",
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error(
            "confirmation mismatch; this tool sends torque-disable and "
            "PREPARE_SHADOW only, never ARM/ENABLE/SETPOINT"
        )
    if args.output.exists():
        parser.error(f"refusing to overwrite existing artifact: {args.output}")
    return args


def load_raw_limits(path: Path) -> tuple[dict[str, int | str], ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        document.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or document.get("operator_approved") is not True
        or document.get("firmware_limit_authorized") is not True
    ):
        raise ValueError("operational limits are not approved")
    records = []
    for side in ("left", "right"):
        arm = document["arms"][side]
        for short_name in (
            "base",
            "shoulder",
            "elbow",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ):
            item = arm[short_name]
            records.append({
                "joint_name": f"{side}_{short_name}_joint",
                "minimum_unwrapped_raw": int(item["minimum_unwrapped_raw"]),
                "maximum_unwrapped_raw": int(item["maximum_unwrapped_raw"]),
            })
    if tuple(item["joint_name"] for item in records) != CANONICAL_JOINT_NAMES:
        raise ValueError("operational-limit joint order mismatch")
    return tuple(records)


def analyze_snapshot(
    positions_raw: tuple[int, ...],
    raw_limits: tuple[dict[str, int | str], ...],
) -> tuple[list[dict[str, object]], int | None]:
    if len(positions_raw) != EXPECTED_JOINT_COUNT:
        raise ValueError("snapshot must contain 12 raw positions")
    if len(raw_limits) != EXPECTED_JOINT_COUNT:
        raise ValueError("raw limits must contain 12 joints")
    output: list[dict[str, object]] = []
    first_failure: int | None = None
    for index, (raw, limit) in enumerate(
        zip(positions_raw, raw_limits, strict=True)
    ):
        candidates = [
            int(raw) + turn * RAW_MODULUS
            for turn in (-1, 0, 1)
            if int(limit["minimum_unwrapped_raw"])
            <= int(raw) + turn * RAW_MODULUS
            <= int(limit["maximum_unwrapped_raw"])
        ]
        reliable = first_failure is None
        passed = len(candidates) == 1
        output.append({
            "index": index,
            "joint_name": limit["joint_name"],
            "modulo_raw": int(raw),
            "minimum_unwrapped_raw": int(limit["minimum_unwrapped_raw"]),
            "maximum_unwrapped_raw": int(limit["maximum_unwrapped_raw"]),
            "matching_unwrapped_raw": candidates,
            "unique_binding": passed,
            "reliable_from_partial_failure_snapshot": reliable,
        })
        if reliable and not passed:
            first_failure = index
    return output, first_failure


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    args = parse_args()
    raw_limits = load_raw_limits(args.limits)
    device = resolve_serial_device(args.device)
    port = open_exclusive_serial(serial, device, HOST_BAUD, timeout_s=0.4)
    try:
        transport = StreamValidationTransportV2(port, response_timeout_s=0.4)
        hello = transport.enter_binary_mode()
        if (
            hello.protocol_version != EXPECTED_PROTOCOL_VERSION
            or hello.joint_count != EXPECTED_JOINT_COUNT
            or hello.firmware_version != F8_FIRMWARE_VERSION
            or hello.left_calibration_hash != CALIBRATION_HASH
            or hello.right_calibration_hash != CALIBRATION_HASH
            or hello.capabilities != F8_CAPABILITIES
        ):
            raise RuntimeError(f"unexpected F8.9 identity: {hello}")
        if hello.stop_latched:
            raise RuntimeError(
                "STM32 STOP is latched; physically RESET the STM32 once, "
                "then run this diagnostic instead of the resident launch"
            )
        snapshot = transport.prepare_shadow()
    finally:
        port.close()

    if snapshot.status_code in (0, 8):
        analysis, first_failure = analyze_snapshot(
            tuple(snapshot.positions_raw), raw_limits
        )
    else:
        analysis, first_failure = [], None
    document = {
        "schema_version": 1,
        "record_kind": "bimanual_anchor_limit_no_motion_diagnostic",
        "motion_commands": 0,
        "arm_enable_commands": 0,
        "setpoint_commands": 0,
        "operations": ["HELLO", "PREPARE_SHADOW"],
        "device": device,
        "baud": HOST_BAUD,
        "hello": asdict(hello),
        "snapshot": asdict(snapshot),
        "limits_path": str(args.limits),
        "limits_sha256": sha256_file(args.limits),
        "analysis": analysis,
        "first_failed_index": first_failure,
        "first_failed_joint": (
            analysis[first_failure]["joint_name"]
            if first_failure is not None
            else None
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = sha256_file(args.output)

    reliable_count = (
        first_failure + 1 if first_failure is not None else len(analysis)
    )
    for item in analysis[:reliable_count]:
        print(
            "ANCHOR_LIMIT_JOINT "
            f"index={item['index']} joint={item['joint_name']} "
            f"raw={item['modulo_raw']} "
            f"range={item['minimum_unwrapped_raw']}.."
            f"{item['maximum_unwrapped_raw']} "
            f"matches={item['matching_unwrapped_raw']} "
            f"pass={str(item['unique_binding']).lower()}"
        )

    if snapshot.status_code == 0 and first_failure is None:
        print(
            "BIMANUAL_ANCHOR_LIMIT_DIAGNOSTIC_PASS "
            f"output={args.output} sha256={digest}"
        )
        return 0
    print(
        "BIMANUAL_ANCHOR_LIMIT_DIAGNOSTIC_FAIL "
        f"firmware_status={snapshot.status_code} "
        f"first_failed_index={first_failure} "
        f"first_failed_joint={document['first_failed_joint']} "
        f"output={args.output} sha256={digest}"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
