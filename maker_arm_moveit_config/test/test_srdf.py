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
    # MoveIt Servo drives from work and stand. It halts at ready and home (singularities).
    # stand keeps link_006 at 0 (the gripper in the arm's plane). work keeps it at -0.1: at 0,
    # Servo halted for a singularity on -x.
    assert arm["work"] == dict(zip(ARM, (0.0, -0.95, 1.2, -0.7, -0.1, 0.0)))
    assert arm["stand"] == dict(zip(ARM, (0.0, -1.6, 1.6, -1.25, 0.0, 0.0)))
    # 0.5 mm inside the jaw limits: an empty close lands a few um past its target.
    jaw = LIMITS["gripper_left_joint"]
    want = {"open": jaw["max_position"] - 0.0005, "closed": jaw["min_position"] + 0.0005}
    for name, value in want.items():
        assert abs(states("gripper")[name]["gripper_left_joint"] - value) < 1e-9, name


def test_the_servo_states_come_first():
    # Conduit offers the arm's states as buttons in SRDF order.
    assert list(states("arm")) == ["work", "stand", "ready", "home"]


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


# Servo scales and halts near its joint bound (MoveIt limit plus joint_limit_margins). A 1 s push
# at 0.05 m/s moved link_005 toward its limit by up to 0.15 rad (-1.3 reported JOINT_BOUND
# at the end of a push, -1.25 never did), so the Servo-safe states keep this travel beyond the bound.
SERVO_TRAVEL_HEADROOM = 0.2
SERVO_STATES = ("work", "stand")  # ready and home are singularities: Servo halts there anyway


def servo_margins():
    margins = yaml.safe_load((PKG / "config" / "servo.yaml").read_text())["joint_limit_margins"]
    return margins * len(ARM) if len(margins) == 1 else margins


def test_servo_keeps_the_singularity_and_margin_settings_the_poses_were_measured_with():
    servo = yaml.safe_load((PKG / "config" / "servo.yaml").read_text())
    assert servo["hard_stop_singularity_threshold"] == 30.0
    assert servo["lower_singularity_threshold"] == 17.0
    assert len(servo_margins()) == len(ARM)


def test_the_servo_states_keep_clear_of_servos_joint_bound():
    margins = dict(zip((f"link_00{i}_joint" for i in range(2, 8)), servo_margins()))
    arm = states("arm")
    for name in SERVO_STATES:
        for j, v in arm[name].items():
            clear = margins[j] + SERVO_TRAVEL_HEADROOM - 1e-9
            lo, hi = LIMITS[j]["min_position"], LIMITS[j]["max_position"]
            assert v - lo >= clear and hi - v >= clear, (name, j, v)


def test_collisions_were_generated():
    assert len(ROOT.findall("disable_collisions")) > 20


def test_the_fingers_may_touch():
    pairs = {frozenset((d.get("link1"), d.get("link2"))) for d in ROOT.findall("disable_collisions")}
    assert frozenset(("gripper_left", "gripper_right")) in pairs
