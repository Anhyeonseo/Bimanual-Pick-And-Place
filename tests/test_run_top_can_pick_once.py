from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
MODULE_PATH = TOOLS / "run_top_can_pick_once.py"
SPEC = importlib.util.spec_from_file_location("run_top_can_pick_once", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
SOURCE = MODULE_PATH.read_text(encoding="utf-8")
SCHEMA14_PLAN = ROOT / "artifacts/can_to_bin/can_pick_plan_run01.json"


def schema17_plan(tmp_path: Path, side: str = "left") -> Path:
    document = json.loads(SCHEMA14_PLAN.read_text(encoding="utf-8"))
    document["schema_version"] = 17
    document["routing"]["nonselected_arm_behavior"] = "hold_current_pose"
    document["routing"]["selected_arm"] = side
    document["joint_names"] = list(
        MODULE.CANONICAL_JOINTS[:5]
        if side == "left"
        else MODULE.CANONICAL_JOINTS[6:11]
    )
    document["gripper_contract"]["commissioned_gripper_sides"] = ["left"]
    document["generated_at_unix_s"] = time.time()
    document["execution_support"] = {
        "supported_by_existing_runner": True,
        "runner": "tools/run_top_can_pick_once.py",
        "reason": (
            "supervised can execution requires an exact plan hash and operator "
            "confirmation; lift, replace, and release are bounded by the "
            "five-second unmonitored contact-hold limit"
        ),
    }
    for endpoint in document["endpoints"].values():
        endpoint["top_down_constraint_applied"] = True
        endpoint["grasp_geometry"].update(
            relationship=(
                "enforced_overhead_downward_jaws_perpendicular_to_can_long_axis"
            ),
            desired_approach_axis=[0.0, 0.0, -1.0],
            achieved_approach_axis=[0.0, 0.0, -1.0],
            approach_tilt_rad=0.0,
            approach_tilt_bound_rad=MODULE.MAXIMUM_TOP_DOWN_TILT_RAD,
        )
    output = tmp_path / "can_plan.json"
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output


def test_schema17_can_plan_passes_pure_validation(tmp_path: Path) -> None:
    plan_path = schema17_plan(tmp_path)
    plan, digest, age_s = MODULE.load_can_plan(plan_path, "", True)
    assert plan["routing"]["selected_arm"] == "left"
    assert plan["gripper_contract"]["open_target_raw"] == 2500
    assert plan["gripper_contract"]["close_target_raw"] == 2285
    assert digest == MODULE.sha256_file(plan_path)
    assert age_s >= 0.0


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda plan: plan.update(schema_version=16), "schema-17"),
        (
            lambda plan: plan["routing"].update(selected_arm="center"),
            "schema-17",
        ),
        (
            lambda plan: plan["gripper_contract"].update(close_target_raw=2270),
            "gripper contract",
        ),
        (
            lambda plan: plan["gripper_contract"].update(
                maximum_unmonitored_contact_hold_s=6.0
            ),
            "gripper contract",
        ),
    ],
)
def test_can_plan_rejects_contract_drift(
    tmp_path: Path, mutation, message: str
) -> None:
    path = schema17_plan(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    mutation(document)
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(RuntimeError, match=message):
        MODULE.load_can_plan(path, "", True)


def test_validate_only_never_contacts_resident_services(tmp_path: Path) -> None:
    plan_path = schema17_plan(tmp_path)
    output = tmp_path / "validation.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(MODULE_PATH),
            "--plan",
            str(plan_path),
            "--validate-only",
            "--output",
            str(output),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["overall_verdict"] == "TOP_CAN_PICK_VALIDATE_ONLY_PASS"
    assert result["resident_services_called"] == 0
    assert result["motion_commands"] == 0


def test_runtime_has_bounded_release_no_retry_and_exact_confirmation() -> None:
    assert MODULE.CONFIRMATION in SOURCE
    assert MODULE.HEIGHT_CHECK_CONFIRMATION in SOURCE
    assert '"automatic_retry_count": 0' in SOURCE
    assert "MAXIMUM_HOLD_S = 5.0" in SOURCE
    assert "MAXIMUM_PLANNED_RELEASE_ROUTE_S = 4.0" in SOURCE
    assert "MAXIMUM_APPROACH_CHECKPOINTS_PER_ACTION = 2" in SOURCE
    assert "EXPECTED_OPEN_RAW = 2500" in SOURCE
    assert "EXPECTED_CLOSE_RAW = 2285" in SOURCE
    assert "EXPECTED_CONTACT_RESIDUAL_RANGE = (19, 23)" in SOURCE
    assert "OPEN_RESIDUAL_TOLERANCE_RAW = 20" in SOURCE
    assert "failure_emergency_release" in SOURCE
    assert "stop_request_for_owner(OWNER)" in SOURCE
    assert "except KeyboardInterrupt:" in SOURCE
    assert "if force_stop or not resident_ready or object_may_be_held:" in SOURCE
    assert "serial.Serial" not in SOURCE
    assert "for retry" not in SOURCE
    assert "while retry" not in SOURCE


def test_long_approach_is_split_into_short_resident_actions() -> None:
    assert "for chunk_start in range(" in SOURCE
    assert "MAXIMUM_APPROACH_CHECKPOINTS_PER_ACTION" in SOURCE
    assert '"approach_checkpoints_"' in SOURCE


def test_can_runner_zeros_only_the_dynamically_selected_arm() -> None:
    start = tuple(float(index + 1) / 10.0 for index in range(12))
    left = MODULE.selected_arm_q0_target(start, "left")
    right = MODULE.selected_arm_q0_target(start, "right")
    assert left[:5] == (0.0,) * 5
    assert left[5:] == start[5:]
    assert right[:6] == start[:6]
    assert right[6:11] == (0.0,) * 5
    assert right[11] == start[11]
    assert "bimanual_q0_target" not in SOURCE
    assert '"nonselected_arm_motion_commanded"] = False' in SOURCE


def test_right_can_plan_validates_but_physical_execution_is_gated(
    tmp_path: Path,
) -> None:
    plan_path = schema17_plan(tmp_path, side="right")
    plan, _, _ = MODULE.load_can_plan(plan_path, "", True)
    assert plan["routing"]["selected_arm"] == "right"
    with pytest.raises(RuntimeError, match="right can gripper is not commissioned"):
        MODULE.load_can_plan(plan_path, "", False)


def test_height_check_has_no_close_command_and_returns_to_q0() -> None:
    assert "TOP_CAN_OPEN_GRASP_HEIGHT_CHECK_HOLD" in SOURCE
    assert "close_commands=0" in SOURCE
    height_branch = SOURCE[
        SOURCE.index("if args.open_grasp_height_check:"):
        SOURCE.index("        else:", SOURCE.index("if args.open_grasp_height_check:"))
    ]
    assert "pick_close" not in height_branch
    assert "open_grasp_to_pregrasp_to_q0" in height_branch


def test_shared_resident_helpers_allow_a_dedicated_owner() -> None:
    shared = (TOOLS / "run_top_pick_place_application_once.py").read_text(
        encoding="utf-8"
    )
    assert "owner: str = OWNER" in shared
    assert "request.owner = owner" in shared
    assert 'document.get("owner") == owner' in shared
