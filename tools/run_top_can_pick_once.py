#!/usr/bin/env python3
"""Run one supervised dynamically routed lying-can pick and release.

The input is a fresh, SHA-pinned schema-17 plan produced by
``plan_top_camera_pick_place_once.py --object-profile can --task pick-only``.
Perception and planning never authorize motion by themselves.  Execution needs
an exact operator confirmation, has no retry path, and releases the can within
the plan's five-second unmonitored contact-hold limit.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
for source_path in (
    ROOT / "tools",
    ROOT / "ros2_ws" / "src" / "single_arm_bridge",
):
    if str(source_path) not in sys.path:
        sys.path.insert(0, str(source_path))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState  # noqa: E402
from so101_interfaces.msg import BimanualJointFeedback  # noqa: E402
from so101_interfaces.srv import BimanualStreamCommand  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from run_top_pick_place_application_once import (  # noqa: E402
    call,
    continuous_finite_request,
    feedback_positions,
    response_document,
    status_document,
    status_prepared_positions,
    stop_request_for_owner,
    terminal_anchor_settle_evidence,
    wait_for_anchor,
    wait_until_ready,
)
from top_pick_place_application import (  # noqa: E402
    ARM_TERMINAL_TOLERANCE_RAD,
    CANONICAL_JOINTS,
    RAW_STEP_RAD,
    residual_raw,
    sha256_file,
    step_target,
    validate_selected_q0_anchor,
)


CONFIRMATION = "RUN_TOP_CAN_PICK_LIFT_REPLACE_RELEASE_ONCE"
HEIGHT_CHECK_CONFIRMATION = "RUN_TOP_CAN_OPEN_GRASP_HEIGHT_CHECK_ONCE"
OWNER = "top_can_pick_application"
STATUS_SERVICE = "/bimanual_stream_adapter/status"
REFRESH_ANCHOR_SERVICE = "/bimanual_stream_adapter/refresh_anchor"
COMMAND_SERVICE = "/bimanual_stream_adapter/command"
ANCHOR_TOPIC = "/bimanual_stream_adapter/anchor_joint_states"
FEEDBACK_TOPIC = "/bimanual_stream_adapter/feedback"
EXPECTED_FIRMWARES = frozenset(("0x00024809",))
EXPECTED_SCHEMA_VERSION = 17
EXPECTED_PLAN_STATUS = "DYNAMIC_TOP_CAN_PICK_PLAN_ONLY_PASS"
SUPPORTED_ARMS = frozenset(("left", "right"))
COMMISSIONED_GRIPPER_SIDES = frozenset(("left",))
EXPECTED_OPEN_RAW = 2500
EXPECTED_CLOSE_RAW = 2285
EXPECTED_OPEN_RAD = (2048 - EXPECTED_OPEN_RAW) * RAW_STEP_RAD
EXPECTED_CLOSE_RAD = (2048 - EXPECTED_CLOSE_RAW) * RAW_STEP_RAD
EXPECTED_CONTACT_RESIDUAL_RANGE = (19, 23)
# The commissioned open target normally settles within 6 raw. The physical
# height-check run on 2026-08-17 settled at 18 raw while the jaws remained
# visibly clear of the 53 mm can. Keep a small margin without weakening the
# separate 19..23 raw contact-residual contract used to prove a grasp.
OPEN_RESIDUAL_TOLERANCE_RAW = 20
MAXIMUM_PLAN_AGE_S = 300.0
MAXIMUM_HOLD_S = 5.0
MAXIMUM_PLANNED_RELEASE_ROUTE_S = 4.0
MAXIMUM_ENDPOINT_RESIDUAL_M = 0.0021
MAXIMUM_CROSSING_RESIDUAL_RAD = math.radians(2.0)
MAXIMUM_TOP_DOWN_TILT_RAD = math.radians(20.0)
MAXIMUM_ARM_STEP_RAD = 0.18
MAXIMUM_APPROACH_CHECKPOINTS_PER_ACTION = 2
MAXIMUM_FALLBACK_FEEDBACK_AGE_MS = 150
OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
CALIBRATION = ROOT / "config/single_arm_calibration.json"
REQUIRED_ENDPOINTS = frozenset(("pick_pregrasp", "pick_grasp", "pick_lift"))
EXPECTED_PHASES = (
    "q0_to_pick_pregrasp",
    "pick_pregrasp_to_grasp",
    "pick_grasp_to_lift",
)
ARM_INDICES = {
    "left": (0, 1, 2, 3, 4),
    "right": (6, 7, 8, 9, 10),
}
ARM_WITH_GRIPPER_INDICES = {
    "left": tuple(range(6)),
    "right": tuple(range(6, 12)),
}
GRIPPER_INDEX = {"left": 5, "right": 11}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--plan-sha256", default="")
    parser.add_argument("--confirmation", default="")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--open-grasp-height-check", action="store_true")
    parser.add_argument("--hold-at-grasp-s", type=float, default=2.0)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if not 0.0 <= args.hold_at_grasp_s <= MAXIMUM_HOLD_S:
        parser.error(f"--hold-at-grasp-s must be within 0..{MAXIMUM_HOLD_S:g}")
    if args.validate_only and args.open_grasp_height_check:
        parser.error("--validate-only and --open-grasp-height-check are exclusive")
    if not args.validate_only:
        expected = (
            HEIGHT_CHECK_CONFIRMATION
            if args.open_grasp_height_check
            else CONFIRMATION
        )
        if args.confirmation != expected:
            parser.error(f"execution requires --confirmation {expected}")
        if len(args.plan_sha256) != 64 or any(
            char not in "0123456789abcdefABCDEF" for char in args.plan_sha256
        ):
            parser.error("execution requires a 64-character --plan-sha256")
    return args


def _finite_vector(value, count: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != count:
        raise RuntimeError(f"{label} must contain {count} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f"{label} contains a non-finite value")
    return result


def _close(value: float, expected: float, *, tolerance: float = 1e-9) -> bool:
    return math.isclose(float(value), expected, abs_tol=tolerance)


def selected_arm_q0_target(
    current: tuple[float, ...], side: str
) -> tuple[float, ...]:
    """Zero only the selected arm while preserving both grippers and its peer."""
    if len(current) != 12:
        raise RuntimeError("selected-arm q0 target requires twelve positions")
    if side not in SUPPORTED_ARMS:
        raise RuntimeError(f"unsupported selected arm: {side}")
    target = list(float(value) for value in current)
    for index in ARM_INDICES[side]:
        target[index] = 0.0
    return tuple(target)


def _load_arm_bounds(
    path: Path, side: str
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    limits = json.loads(path.read_text(encoding="utf-8"))
    if (
        limits.get("record_kind") != "bimanual_operational_limits"
        or limits.get("operator_approved") is not True
        or limits.get("firmware_limit_authorized") is not True
    ):
        raise RuntimeError("operational limit file is not operator approved")
    names = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
    arm = limits.get("arms", {}).get(side, {})
    lower = tuple(float(arm[name]["minimum_urad"]) / 1.0e6 for name in names)
    upper = tuple(float(arm[name]["maximum_urad"]) / 1.0e6 for name in names)
    return lower, upper


def partition_can_steps(plan: dict) -> tuple[dict, list[dict], dict, list[dict]]:
    """Validate and split a variable-segment can route by semantic phase."""
    steps = plan.get("steps")
    if not isinstance(steps, list) or len(steps) < 7:
        raise RuntimeError("can command sequence is incomplete")
    if [step.get("index") for step in steps] != list(range(1, len(steps) + 1)):
        raise RuntimeError("can command step indices are invalid")
    if steps[0].get("kind") != "gripper" or steps[0].get("phase") != "pick_open":
        raise RuntimeError("can command sequence must begin with pick_open")
    close_indices = [
        index
        for index, step in enumerate(steps)
        if step.get("kind") == "gripper" and step.get("phase") == "pick_close"
    ]
    if len(close_indices) != 1:
        raise RuntimeError("can command sequence requires one pick_close")
    close_index = close_indices[0]
    approach = steps[1:close_index]
    lift = steps[close_index + 1 :]
    if (
        not approach
        or not lift
        or any(step.get("kind") != "arm" for step in approach + lift)
        or tuple(dict.fromkeys(step.get("phase") for step in approach))
        != ("q0_to_pick_pregrasp", "pick_pregrasp_to_grasp")
        or any(step.get("phase") != "pick_grasp_to_lift" for step in lift)
    ):
        raise RuntimeError("can approach/close/lift phase ordering is invalid")
    return steps[0], approach, steps[close_index], lift


def load_can_plan(path: Path, expected_sha256: str, validate_only: bool):
    actual_sha256 = sha256_file(path)
    if expected_sha256 and actual_sha256 != expected_sha256.lower():
        raise RuntimeError(
            "can plan sha256 mismatch: "
            f"expected={expected_sha256.lower()} actual={actual_sha256}"
        )
    plan = json.loads(path.read_text(encoding="utf-8"))
    routing = plan.get("routing", {})
    support = plan.get("execution_support", {})
    selected_arm = routing.get("selected_arm")
    if (
        plan.get("schema_version") != EXPECTED_SCHEMA_VERSION
        or plan.get("status") != EXPECTED_PLAN_STATUS
        or plan.get("object_profile") != "can"
        or plan.get("task") != "pick-only"
        or plan.get("execution_api_used") is not False
        or plan.get("motion_authorized") is not False
        or plan.get("automatic_execution_permitted") is not False
        or plan.get("terminal_behavior")
        != "bounded_hold_at_pick_lift_release_required_within_5s"
        or selected_arm not in SUPPORTED_ARMS
        or routing.get("rule") != "source_image_center_x"
        or routing.get("nonselected_arm_behavior") != "hold_current_pose"
        or support.get("supported_by_existing_runner") is not True
        or support.get("runner") != "tools/run_top_can_pick_once.py"
    ):
        raise RuntimeError("schema-17 supervised can plan contract is invalid")
    expected_joints = tuple(
        CANONICAL_JOINTS[:5]
        if selected_arm == "left"
        else CANONICAL_JOINTS[6:11]
    )
    if tuple(plan.get("joint_names", ())) != expected_joints:
        raise RuntimeError(
            f"can plan must target the {selected_arm} arm joint order"
        )
    if plan.get("q0_rad") != [0.0] * 5:
        raise RuntimeError("can plan q0 contract is invalid")

    calibration = plan.get("calibration", {})
    operational = plan.get("operational_limits", {})
    if (
        calibration.get("sha256") != sha256_file(CALIBRATION)
        or operational.get("sha256") != sha256_file(OPERATIONAL_LIMITS)
    ):
        raise RuntimeError("can plan calibration or operational-limit hash mismatch")

    geometry = plan.get("object_geometry", {})
    if (
        geometry.get("state") != "lying"
        or not _close(geometry.get("length_m", math.nan), 0.12244)
        or not _close(geometry.get("diameter_m", math.nan), 0.053)
        or not _close(geometry.get("mass_kg", math.nan), 0.013)
        or geometry.get("grasp_strategy")
        != "close_across_body_perpendicular_to_long_axis"
    ):
        raise RuntimeError("lying-can geometry contract is invalid")

    gripper = plan.get("gripper_contract", {})
    if (
        gripper.get("preopen_required") is not True
        or gripper.get("open_phase") != "before_approach"
        or int(gripper.get("open_target_raw", -1)) != EXPECTED_OPEN_RAW
        or int(gripper.get("close_target_raw", -1)) != EXPECTED_CLOSE_RAW
        or not _close(gripper.get("open_target_rad", math.nan), EXPECTED_OPEN_RAD)
        or not _close(gripper.get("close_target_rad", math.nan), EXPECTED_CLOSE_RAD)
        or gripper.get("expected_held_residual_raw_range")
        != list(EXPECTED_CONTACT_RESIDUAL_RANGE)
        or int(gripper.get("open_observed_residual_raw", -1)) != 6
        or gripper.get("automatic_retry_count") != 0
        or gripper.get("continuous_load_current_monitoring_available") is not False
        or not _close(
            gripper.get("maximum_unmonitored_contact_hold_s", math.nan),
            MAXIMUM_HOLD_S,
        )
        or gripper.get("release_on_hold_timeout_required") is not True
        or gripper.get("firmware_position_or_torque_limits_modified") is not False
        or gripper.get("commissioned_gripper_sides")
        != sorted(COMMISSIONED_GRIPPER_SIDES)
    ):
        raise RuntimeError("commissioned can gripper contract is invalid")

    endpoints = plan.get("endpoints")
    if not isinstance(endpoints, dict) or set(endpoints) != REQUIRED_ENDPOINTS:
        raise RuntimeError("can endpoint set is invalid")
    lower, upper = _load_arm_bounds(OPERATIONAL_LIMITS, selected_arm)
    for name in REQUIRED_ENDPOINTS:
        endpoint = endpoints[name]
        positions = _finite_vector(
            endpoint.get("final_joint_positions_rad"), 5, f"{name} endpoint"
        )
        geometry_contract = endpoint.get("grasp_geometry", {})
        if (
            endpoint.get("orientation_constraint_applied") is not True
            or endpoint.get("top_down_constraint_applied") is not True
            or endpoint.get("wrist_roll_yaw_correction_applied") is not True
            or endpoint.get("wrist_roll_policy") != "solve_can_crossing_yaw"
            or endpoint.get("wrist_roll_locked") is not False
            or geometry_contract.get("relationship")
            != "enforced_overhead_downward_jaws_perpendicular_to_can_long_axis"
            or geometry_contract.get("desired_approach_axis") != [0.0, 0.0, -1.0]
            or float(geometry_contract.get("approach_tilt_rad", math.inf))
            > MAXIMUM_TOP_DOWN_TILT_RAD
            or not _close(
                geometry_contract.get("approach_tilt_bound_rad", math.nan),
                MAXIMUM_TOP_DOWN_TILT_RAD,
            )
            or abs(float(geometry_contract.get("crossing_residual_rad", math.inf)))
            > MAXIMUM_CROSSING_RESIDUAL_RAD
            or float(endpoint.get("plan_residual_norm_m", math.inf))
            > MAXIMUM_ENDPOINT_RESIDUAL_M
            or any(
                value < minimum or value > maximum
                for value, minimum, maximum in zip(
                    positions, lower, upper, strict=True
                )
            )
        ):
            raise RuntimeError(f"can orientation or limit contract is invalid: {name}")

    phases = plan.get("phases")
    if (
        not isinstance(phases, list)
        or tuple(phase.get("name") for phase in phases) != EXPECTED_PHASES
    ):
        raise RuntimeError("can phase sequence is invalid")
    open_step, approach_steps, close_step, lift_steps = partition_can_steps(plan)
    if (
        not _close(open_step.get("target_position_rad", math.nan), EXPECTED_OPEN_RAD)
        or not _close(close_step.get("target_position_rad", math.nan), EXPECTED_CLOSE_RAD)
    ):
        raise RuntimeError("can open/approach/close/lift sequence is invalid")
    previous = (0.0,) * 5
    for step in approach_steps + lift_steps:
        positions = _finite_vector(
            step.get("target_positions_rad"), 5, f"step {step['index']}"
        )
        if max(abs(end - start) for start, end in zip(previous, positions, strict=True)) > MAXIMUM_ARM_STEP_RAD + 1e-9:
            raise RuntimeError(f"can arm step exceeds {MAXIMUM_ARM_STEP_RAD} rad")
        previous = positions

    generated_at = float(plan.get("generated_at_unix_s", 0.0))
    age_s = time.time() - generated_at
    if not validate_only and (age_s < 0.0 or age_s > MAXIMUM_PLAN_AGE_S):
        raise RuntimeError(
            f"can plan is stale: age={age_s:.1f}s limit={MAXIMUM_PLAN_AGE_S:.1f}s"
        )
    if not validate_only and selected_arm not in COMMISSIONED_GRIPPER_SIDES:
        raise RuntimeError(
            f"{selected_arm} can gripper is not commissioned for physical execution"
        )
    return plan, actual_sha256, age_s


def _duration_s(request: BimanualStreamCommand.Request) -> float:
    final = request.points[-1].time_from_start
    return final.sec + final.nanosec / 1e9


def _write_result(path: Path, result: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return sha256(path.read_bytes()).hexdigest()


def main() -> int:
    args = parse_args()
    plan, plan_sha256, plan_age_s = load_can_plan(
        args.plan, args.plan_sha256, args.validate_only
    )
    selected_arm = str(plan["routing"]["selected_arm"])
    open_step, approach_steps, close_step, lift_steps = partition_can_steps(plan)
    result = {
        "schema_version": 1,
        "record_kind": "top_can_pick_lift_replace_release_once",
        "validate_only": bool(args.validate_only),
        "open_grasp_height_check": bool(args.open_grasp_height_check),
        "operator_confirmation": args.confirmation,
        "plan": str(args.plan),
        "plan_sha256": plan_sha256,
        "plan_age_s": plan_age_s,
        "selected_arm": selected_arm,
        "target_lock": plan["target_lock"],
        "automatic_retry_count": 0,
        "motion_commands": 0,
        "legs": [],
        "release": {
            "required_within_s": MAXIMUM_HOLD_S,
            "proven": False,
        },
    }
    if args.validate_only:
        result["resident_services_called"] = 0
        result["overall_verdict"] = "TOP_CAN_PICK_VALIDATE_ONLY_PASS"
        digest = _write_result(args.output, result)
        print(
            "TOP_CAN_PICK_VALIDATE_ONLY_PASS motion_commands=0 "
            f"plan_sha256={plan_sha256} output={args.output} sha256={digest}"
        )
        return 0

    rclpy.init()
    node = Node("top_can_pick_once")
    transient_qos = QoSProfile(depth=1)
    transient_qos.reliability = ReliabilityPolicy.RELIABLE
    transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    feedback_qos = QoSProfile(depth=20)
    feedback_qos.reliability = ReliabilityPolicy.RELIABLE
    anchors: list[JointState] = []
    feedback_messages: list[BimanualJointFeedback] = []
    node.create_subscription(JointState, ANCHOR_TOPIC, anchors.append, transient_qos)
    node.create_subscription(
        BimanualJointFeedback, FEEDBACK_TOPIC, feedback_messages.append, feedback_qos
    )
    status_client = node.create_client(Trigger, STATUS_SERVICE)
    refresh_client = node.create_client(Trigger, REFRESH_ANCHOR_SERVICE)
    command_client = node.create_client(BimanualStreamCommand, COMMAND_SERVICE)

    motion_started = False
    resident_ready = False
    object_may_be_held = False
    release_proven = False
    successful_hold = False
    force_stop = False
    epoch = -1
    commanded: tuple[float, ...] | None = None
    opposite_hold: tuple[float, ...] | None = None

    def execute_targets(label: str, targets: list[tuple[float, ...]], indices):
        nonlocal motion_started, resident_ready, epoch, commanded
        assert commanded is not None
        request = continuous_finite_request(commanded, targets, owner=OWNER)
        duration_s = _duration_s(request)
        anchors.clear()
        feedback_messages.clear()
        resident_ready = False
        motion_started = True
        response = call(node, command_client, request, args.timeout_s)
        expected_epoch = epoch + 1
        if (
            not response.accepted
            or response.adapter_state != "active"
            or int(response.arbiter_epoch) != expected_epoch
        ):
            raise RuntimeError(f"{label} request rejected: {response}")
        history, measured, feedback, source = wait_until_ready(
            node,
            status_client,
            anchors,
            feedback_messages,
            epoch=expected_epoch,
            timeout_s=max(args.timeout_s, duration_s + 5.0),
            owner=OWNER,
        )
        resident_ready = True
        settle = terminal_anchor_settle_evidence(
            measured,
            target_positions=targets[-1],
            joint_indices=indices,
            label=label,
            measurement_source=source,
        )
        record = {
            "label": label,
            "epoch": expected_epoch,
            "start_response": response_document(response),
            "trajectory_points": len(request.points),
            "planned_duration_s": duration_s,
            "terminal_positions_rad": list(measured),
            "status_samples": len(history),
            "post_settle": settle,
            "feedback_age_max_ms": (
                max(int(value) for value in feedback.sample_age_ms)
                if feedback is not None
                else None
            ),
        }
        result["legs"].append(record)
        result["motion_commands"] += 1
        epoch = expected_epoch
        commanded = measured
        print(
            f"TOP_CAN_ACTION_PASS label={label} epoch={epoch} "
            f"duration_s={duration_s:.3f} source={source}"
        )
        return record

    try:
        for name, client in (
            (STATUS_SERVICE, status_client),
            (REFRESH_ANCHOR_SERVICE, refresh_client),
            (COMMAND_SERVICE, command_client),
        ):
            if not client.wait_for_service(timeout_sec=args.timeout_s):
                raise RuntimeError(f"service unavailable: {name}")

        initial = status_document(node, status_client, args.timeout_s)
        initial_owner = initial.get("owner")
        epoch = int(initial.get("arbiter_epoch", -1))
        if (
            initial.get("state") != "ready"
            or initial_owner not in (None, OWNER)
            or (initial_owner is None and epoch != 0)
            or initial.get("firmware_version") not in EXPECTED_FIRMWARES
            or initial.get("motion_authorized") is not True
        ):
            raise RuntimeError(f"unexpected resident initial state: {initial}")
        if initial_owner is None:
            response = call(node, refresh_client, Trigger.Request(), args.timeout_s)
            if not response.success:
                raise RuntimeError(f"resident anchor refresh failed: {response}")
            refreshed = json.loads(response.message)
            if "prepared_positions_rad" in refreshed:
                commanded = status_prepared_positions(
                    refreshed,
                    label="startup refresh",
                    expected_epoch=epoch,
                    require_torque_hold=False,
                )
                startup_source = "resident_refresh_service_anchor"
            else:
                commanded, startup_source, feedback = wait_for_anchor(
                    node, anchors, feedback_messages, args.timeout_s
                )
                if feedback is not None and max(feedback.sample_age_ms) > MAXIMUM_FALLBACK_FEEDBACK_AGE_MS:
                    raise RuntimeError("startup fallback feedback is stale")
        else:
            commanded = status_prepared_positions(
                initial,
                label="same-owner startup",
                expected_epoch=epoch,
                require_torque_hold=True,
            )
            startup_source = "resident_armed_status_terminal_anchor"
        result["initial_status"] = initial
        result["startup_anchor_rad"] = list(commanded)
        result["startup_source"] = startup_source
        print(f"TOP_CAN_FRESH_ANCHOR_PASS source={startup_source}")

        arm_indices = ARM_INDICES[selected_arm]
        arm_with_gripper_indices = ARM_WITH_GRIPPER_INDICES[selected_arm]
        gripper_index = GRIPPER_INDEX[selected_arm]
        opposite_hold = commanded[6:] if selected_arm == "left" else commanded[:6]

        q0_target = selected_arm_q0_target(commanded, selected_arm)
        q0_record = execute_targets(
            f"{selected_arm}_arm_q0", [q0_target], arm_indices
        )
        validate_selected_q0_anchor(commanded, selected_arm)
        q0_record["maximum_q0_residual_rad"] = max(
            abs(commanded[index]) for index in arm_indices
        )
        q0_record["nonselected_arm_motion_commanded"] = False

        open_target = step_target(
            commanded, open_step, opposite_hold, arm=selected_arm
        )
        open_record = execute_targets(
            "pick_open", [open_target], arm_with_gripper_indices
        )
        open_residual = residual_raw(EXPECTED_OPEN_RAD, commanded[gripper_index])
        open_record["gripper_residual_raw"] = open_residual
        if open_residual > OPEN_RESIDUAL_TOLERANCE_RAW:
            raise RuntimeError(
                f"{selected_arm} gripper did not open: residual_raw={open_residual}"
            )

        for chunk_start in range(
            0,
            len(approach_steps),
            MAXIMUM_APPROACH_CHECKPOINTS_PER_ACTION,
        ):
            chunk = approach_steps[
                chunk_start :
                chunk_start + MAXIMUM_APPROACH_CHECKPOINTS_PER_ACTION
            ]
            approach_targets: list[tuple[float, ...]] = []
            planned = commanded
            for step in chunk:
                planned = step_target(
                    planned, step, opposite_hold, arm=selected_arm
                )
                approach_targets.append(planned)
            execute_targets(
                "approach_checkpoints_"
                f"{chunk_start + 1}_to_{chunk_start + len(chunk)}",
                approach_targets,
                arm_indices,
            )

        if args.open_grasp_height_check:
            print(
                "TOP_CAN_OPEN_GRASP_HEIGHT_CHECK_HOLD "
                f"seconds={args.hold_at_grasp_s:.1f} close_commands=0"
            )
            time.sleep(args.hold_at_grasp_s)
            pregrasp = step_target(
                commanded,
                {
                    "kind": "arm",
                    "target_positions_rad": plan["endpoints"]["pick_pregrasp"]
                    ["final_joint_positions_rad"],
                },
                opposite_hold,
                arm=selected_arm,
            )
            q0_return = selected_arm_q0_target(pregrasp, selected_arm)
            execute_targets(
                "open_grasp_to_pregrasp_to_q0",
                [pregrasp, q0_return],
                arm_indices,
            )
            successful_hold = True
            result["overall_verdict"] = "TOP_CAN_OPEN_GRASP_HEIGHT_CHECK_PASS_HOLDING"
        else:
            close_target = step_target(
                commanded, close_step, opposite_hold, arm=selected_arm
            )
            close_record = execute_targets(
                "pick_close", [close_target], arm_with_gripper_indices
            )
            # A failed residual check can still mean the jaws touched or held
            # the can.  Mark the object as potentially held before evaluating
            # the commissioned 19..23 raw evidence so every failure path tries
            # to open the gripper.
            object_may_be_held = True
            contact_residual = residual_raw(
                EXPECTED_CLOSE_RAD, commanded[gripper_index]
            )
            close_record["gripper_residual_raw"] = contact_residual
            if not EXPECTED_CONTACT_RESIDUAL_RANGE[0] <= contact_residual <= EXPECTED_CONTACT_RESIDUAL_RANGE[1]:
                raise RuntimeError(
                    "can contact residual outside commissioned range: "
                    f"residual_raw={contact_residual}"
                )
            held_since = time.monotonic()
            result["contact_residual_raw"] = contact_residual
            print(f"TOP_CAN_CONTACT_PASS residual_raw={contact_residual}")

            release_targets: list[tuple[float, ...]] = []
            planned = commanded
            for step in lift_steps:
                planned = step_target(
                    planned, step, opposite_hold, arm=selected_arm
                )
                release_targets.append(planned)
            grasp_again = step_target(
                planned,
                {
                    "kind": "arm",
                    "target_positions_rad": plan["endpoints"]["pick_grasp"]
                    ["final_joint_positions_rad"],
                },
                opposite_hold,
                arm=selected_arm,
            )
            release_targets.append(grasp_again)
            released = step_target(
                grasp_again, open_step, opposite_hold, arm=selected_arm
            )
            release_targets.append(released)
            release_request = continuous_finite_request(
                commanded, release_targets, owner=OWNER
            )
            planned_release_s = _duration_s(release_request)
            if planned_release_s > MAXIMUM_PLANNED_RELEASE_ROUTE_S:
                raise RuntimeError(
                    "planned held-object route is too long: "
                    f"duration={planned_release_s:.3f}s"
                )
            release_record = execute_targets(
                "pick_lift_replace_release",
                release_targets,
                arm_with_gripper_indices,
            )
            hold_elapsed_s = time.monotonic() - held_since
            release_residual = residual_raw(
                EXPECTED_OPEN_RAD, commanded[gripper_index]
            )
            release_record["gripper_residual_raw"] = release_residual
            release_record["contact_hold_elapsed_s"] = hold_elapsed_s
            result["release"].update(
                {
                    "planned_route_s": planned_release_s,
                    "elapsed_s": hold_elapsed_s,
                    "gripper_residual_raw": release_residual,
                }
            )
            if release_residual > OPEN_RESIDUAL_TOLERANCE_RAW:
                raise RuntimeError(
                    f"can release did not settle: residual_raw={release_residual}"
                )
            release_proven = True
            object_may_be_held = False
            result["release"]["proven"] = True
            if hold_elapsed_s > MAXIMUM_HOLD_S:
                raise RuntimeError(
                    f"can contact hold exceeded {MAXIMUM_HOLD_S:.1f}s: "
                    f"elapsed={hold_elapsed_s:.3f}s"
                )
            print(
                "TOP_CAN_RELEASE_PASS "
                f"elapsed_s={hold_elapsed_s:.3f} residual_raw={release_residual}"
            )

            pregrasp = step_target(
                commanded,
                {
                    "kind": "arm",
                    "target_positions_rad": plan["endpoints"]["pick_pregrasp"]
                    ["final_joint_positions_rad"],
                },
                opposite_hold,
                arm=selected_arm,
            )
            q0_return = selected_arm_q0_target(pregrasp, selected_arm)
            execute_targets(
                "released_pregrasp_to_q0",
                [pregrasp, q0_return],
                arm_indices,
            )
            successful_hold = True
            result["overall_verdict"] = "TOP_CAN_PICK_LIFT_REPLACE_RELEASE_PASS_HOLDING"

        final = status_document(node, status_client, args.timeout_s)
        if (
            final.get("state") != "ready"
            or final.get("owner") != OWNER
            or int(final.get("arbiter_epoch", -1)) != epoch
        ):
            raise RuntimeError(f"unexpected final resident state: {final}")
        result["final_status"] = final
        result["torque_hold_active"] = True
        digest = _write_result(args.output, result)
        print(
            f"{result['overall_verdict']} motion_commands={result['motion_commands']} "
            f"output={args.output} sha256={digest}"
        )
        return 0
    except KeyboardInterrupt:
        force_stop = True
        result["overall_verdict"] = "TOP_CAN_PICK_ONCE_INTERRUPTED"
        result["failure"] = "KeyboardInterrupt: operator requested immediate stop"
        print("TOP_CAN_PICK_ONCE_INTERRUPTED action=request_resident_stop")
        return 130
    except Exception as error:
        result["overall_verdict"] = "TOP_CAN_PICK_ONCE_FAIL"
        result["failure"] = f"{type(error).__name__}: {error}"
        print(f"TOP_CAN_PICK_ONCE_FAIL reason={result['failure']}")
        return 2
    finally:
        if motion_started and not successful_hold:
            if object_may_be_held and resident_ready and commanded is not None and opposite_hold is not None:
                try:
                    emergency_open = step_target(
                        commanded,
                        {
                            "kind": "gripper",
                            "target_position_rad": EXPECTED_OPEN_RAD,
                        },
                        opposite_hold,
                        arm=selected_arm,
                    )
                    execute_targets(
                        "failure_emergency_release",
                        [emergency_open],
                        arm_with_gripper_indices,
                    )
                    emergency_residual = residual_raw(
                        EXPECTED_OPEN_RAD, commanded[gripper_index]
                    )
                    release_proven = emergency_residual <= OPEN_RESIDUAL_TOLERANCE_RAW
                    result["release"].update(
                        {
                            "emergency_attempted": True,
                            "emergency_residual_raw": emergency_residual,
                            "proven": release_proven,
                        }
                    )
                    object_may_be_held = not release_proven
                except Exception as release_error:
                    result["release"]["emergency_error"] = (
                        f"{type(release_error).__name__}: {release_error}"
                    )
            if force_stop or not resident_ready or object_may_be_held:
                try:
                    stopped = call(
                        node,
                        command_client,
                        stop_request_for_owner(OWNER),
                        min(args.timeout_s, 2.0),
                    )
                    result["failure_stop_response"] = response_document(stopped)
                except Exception as stop_error:
                    result["failure_stop_error"] = (
                        f"{type(stop_error).__name__}: {stop_error}"
                    )
        if result:
            _write_result(args.output, result)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
