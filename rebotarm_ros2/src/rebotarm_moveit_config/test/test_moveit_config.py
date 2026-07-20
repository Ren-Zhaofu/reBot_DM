from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
JOINTS = [f"joint{i}" for i in range(1, 7)]


def test_srdf_has_arm_chain_and_six_joint_zero_state() -> None:
    root = ET.parse(ROOT / "config" / "rebotarm.srdf").getroot()
    arm = next(group for group in root.findall("group") if group.get("name") == "arm")
    chain = arm.find("chain")
    assert chain is not None
    assert chain.get("base_link") == "base_link"
    assert chain.get("tip_link") == "end_link"
    zero = next(
        state for state in root.findall("group_state") if state.get("name") == "zero"
    )
    assert [joint.get("name") for joint in zero.findall("joint")] == JOINTS


def test_moveit_controller_maps_to_driver_action() -> None:
    data = yaml.safe_load((ROOT / "config" / "moveit_controllers.yaml").read_text())
    manager = data["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["rebotarm"]
    assert manager["rebotarm"]["action_ns"] == "follow_joint_trajectory"
    assert manager["rebotarm"]["joints"] == JOINTS


def test_moveit_source_has_no_machine_specific_absolute_path() -> None:
    for path in ROOT.rglob("*"):
        if path.is_file() and "test" not in path.parts:
            text = path.read_text(errors="ignore")
            assert "/home/" not in text
            assert "/Desktop/" not in text
