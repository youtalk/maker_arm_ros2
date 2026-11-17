import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import xacro
import yaml
from urdf_parser_py.urdf import URDF

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG.parent / "maker_arm_description" / "urdf" / "maker_arm.urdf.xacro"
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


def test_the_jaw_states_sit_0p5_mm_inside_the_jaw_limits():
    # An empty close lands a few um past its target.
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


def test_home_is_the_vendor_zero_pulled_inside_the_moveit_limits():
    for j, v in states("arm")["home"].items():
        lo, hi = LIMITS[j]["min_position"], LIMITS[j]["max_position"]
        assert abs(v - min(max(0.0, lo + 0.05), hi - 0.05)) < 1e-9, j


# Servo scales and halts near its joint bound (MoveIt limit plus joint_limit_margins). A 1 s push
# at 0.05 m/s moved link_005 toward its limit by up to 0.15 rad (-1.3 reported JOINT_BOUND
# at the end of a push, -1.25 never did), so the Servo-safe states keep this travel beyond the bound.
SERVO_TRAVEL_HEADROOM = 0.2
SERVO_STATES = ("work", "stand")  # ready sits inside the link_005 margin, home is singular


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


# fk() as in maker_arm_description's test_description.py.
def _rot(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis, q):
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def fk(robot, link, q):
    """Pose (4x4) of `link` in the robot's root link."""
    parent_joint = {j.child: j for j in robot.joints}
    t = np.eye(4)
    while link in parent_joint:
        j = parent_joint[link]
        o = j.origin
        local = np.eye(4)
        local[:3, :3] = _rot(o.rpy if o else (0, 0, 0))
        local[:3, 3] = o.xyz if o else (0, 0, 0)
        if j.type == "revolute":
            m = np.eye(4)
            m[:3, :3] = _axis_angle(j.axis, q.get(j.name, 0.0))
            local = local @ m
        t = local @ t
        link = j.parent
    return t


ROBOT = URDF.from_xml_string(
    xacro.process_file(str(XACRO), mappings={"use_ros2_control": "false"}).toxml())


def grasp_center(joints):
    return fk(ROBOT, "grasp_center", joints)  # in mount_link, the root


def jacobian(joints, h=1e-6):
    """Servo's 6x6 Jacobian of grasp_center in mount_link: linear rows, then angular rows."""
    rot = grasp_center(joints)[:3, :3]
    cols = []
    for name in ARM:
        plus = grasp_center({**joints, name: joints[name] + h})
        minus = grasp_center({**joints, name: joints[name] - h})
        v = (plus[:3, 3] - minus[:3, 3]) / (2 * h)
        w = (plus[:3, :3] - minus[:3, :3]) / (2 * h) @ rot.T  # the skew matrix of the angular rate
        cols.append([*v, w[2, 1], w[0, 2], w[1, 0]])
    return np.array(cols).T


def test_the_servo_states_sit_between_servos_singularity_thresholds():
    servo = yaml.safe_load((PKG / "config" / "servo.yaml").read_text())
    lo, hi = servo["lower_singularity_threshold"], servo["hard_stop_singularity_threshold"]
    for name in SERVO_STATES:
        c = np.linalg.cond(jacobian(states("arm")[name]))
        assert lo < c < hi, (name, c)


def test_ready_keeps_its_place_and_points_the_tool_as_low_as_j4_allows():
    ready = states("arm")["ready"]
    t = grasp_center(ready)
    # The lab's ready position, in the arm's plane.
    assert np.linalg.norm(t[:3, 3] - (0.2763, 0.0, 0.1607)) < 0.001, t[:3, 3]
    assert ready["link_006_joint"] == 0.0
    # The vendor wrist_flex stop keeps the tool off vertical: link_005 sits one margin inside its
    # MoveIt limit.
    assert abs(ready["link_005_joint"] - (LIMITS["link_005_joint"]["min_position"] + 0.05)) < 1e-4
    # The tool points along -z of grasp_center: its angle to straight down.
    tilt = math.degrees(math.acos(t[2, 2]))
    assert tilt <= 30.0, tilt
