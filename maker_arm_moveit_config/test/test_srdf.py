import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
ROOT = ET.parse(PKG / "config" / "maker_arm.srdf").getroot()
LIMITS = yaml.safe_load((PKG / "config" / "joint_limits.yaml").read_text())["joint_limits"]
ARM = [f"link_00{i}_joint" for i in range(2, 8)]


def group(name):
    return next(g for g in ROOT.findall("group") if g.get("name") == name)


def states(name):
    out = {}
    for s in ROOT.findall("group_state"):
        if s.get("group") == name:
            out[s.get("name")] = {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
    return out


def test_the_arm_group_is_the_mount_to_grasp_center_chain():
    (chain,) = group("arm").findall("chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("mount_link", "grasp_center")
    assert [j.get("name") for j in group("gripper").findall("joint")] == ["gripper_left_joint"]
    (ee,) = ROOT.findall("end_effector")
    assert (ee.get("group"), ee.get("parent_link"), ee.get("parent_group")) == (
        "gripper", "link_007", "arm")


def test_the_named_states():
    arm = states("arm")
    assert arm["ready"] == dict(zip(ARM, (0.0, -1.79, 2.05, -1.3, -0.22, 0.0)))
    for name, value in (("open", 0.0524125), ("closed", 0.0)):
        assert states("gripper")[name] == {"gripper_left_joint": value}


def test_the_named_states_lie_inside_the_moveit_limits():
    for group_name in ("arm", "gripper"):
        for name, joints in states(group_name).items():
            for j, v in joints.items():
                lo, hi = LIMITS[j]["min_position"], LIMITS[j]["max_position"]
                assert lo <= v <= hi, (name, j, v)


def test_home_is_the_zero_pose_pulled_0p1_inside_the_urdf_limits():
    urdf = ET.parse(PKG.parent / "maker_arm_description" / "urdf" / "maker_arm_sdk.urdf")
    for j in urdf.getroot().findall("joint"):
        if j.get("name") in ARM:
            lim = j.find("limit")
            lo, hi = float(lim.get("lower")), float(lim.get("upper"))
            want = min(max(0.0, lo + 0.1), hi - 0.1)
            assert abs(states("arm")["home"][j.get("name")] - want) < 1e-9


def test_collisions_were_generated():
    assert len(ROOT.findall("disable_collisions")) > 20
