from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "tools/package_trained_can_obb_bundle.py"
SPEC = importlib.util.spec_from_file_location("package_trained_can_obb_bundle", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_holdout_selection_excludes_exact_training_bytes() -> None:
    training_manifest = MODULE.load_json(
        ROOT / "datasets/can_obb_auto_v1/autolabel_manifest.json"
    )
    entries = MODULE.selected_holdout_entries(
        ROOT / "datasets/dual_arm_can_disposal_20260815_measured",
        training_manifest,
        32,
        0,
    )
    training_hashes = {
        item["image_sha256"] for item in training_manifest["images"]
    }
    assert len(entries) == 32
    assert sum(item["requires_can_obb"] for item in entries) == 32
    assert not ({item["sha256"] for item in entries} & training_hashes)


def test_bundle_tool_is_explicitly_non_deployment() -> None:
    source = PATH.read_text(encoding="utf-8")
    assert '"motion_authorized": False' in source
    assert '"deployment_acceptance_permitted": False' in source
    assert '"correlated_capture_sessions": True' in source
    assert '"false_positive_acceptance_permitted": False' in source
    assert '"holdout_used_for_training": False' in source
