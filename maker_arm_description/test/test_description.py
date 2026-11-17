import json
import math
import os
import subprocess
from pathlib import Path

import numpy as np
import xacro
from urdf_parser_py.urdf import URDF

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / "urdf" / "maker_arm.urdf.xacro"
DAEMON = PKG / "config" / "daemon"
# The lab's robots/frames.py HOME_JOINT_POS.
LAB_HOME = {
    "link_002_joint": 0.0,
    "link_003_joint": -1.79,
    "link_004_joint": 2.05,
    "link_005_joint": -1.3,
    "link_006_joint": -0.22,
    "link_007_joint": 0.0,
}
# The SRDF's ready: the lab's grasp_center position with link_006 at 0, and link_005 inside the
# vendor wrist_flex stop.
READY = dict(zip(LAB_HOME, (0.0, -1.5518, 1.6901, -1.038, 0.0, 0.0)))
ARM_AND_GRIPPER = [*LAB_HOME, "gripper_left_joint"]
# The owner's folded rest (the P0 scan), in degrees in the URDF frame.
REST = dict(zip(LAB_HOME, (math.radians(d) for d in (-8.0, 0.0, 7.9, -24.5, 2.6, -3.2))))


def render(**mappings):
    return xacro.process_file(str(XACRO), mappings={k: str(v) for k, v in mappings.items()})


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


def robot_of(doc):
    return URDF.from_xml_string(doc.toxml())


def test_the_xacro_renders_with_mount_link_as_root():
    robot = robot_of(render())
    assert robot.name == "maker_arm"
    assert robot.get_root() == "mount_link"
    joint = robot.joint_map["mount_joint"]
    assert (joint.parent, joint.child) == ("mount_link", "base_link")
    assert np.allclose(joint.origin.rpy, (math.pi / 2, 0, math.pi))


def test_the_lab_home_pose_puts_grasp_center_where_the_lab_measured_it():
    robot = robot_of(render())
    pos = fk(robot, "grasp_center", LAB_HOME)[:3, 3]
    assert np.allclose(pos, (0.2749, 0.0371, 0.1646), atol=0.002), pos


def test_the_ready_pose_puts_grasp_center_in_the_arms_plane():
    robot = robot_of(render())
    pos = fk(robot, "grasp_center", READY)[:3, 3]
    assert np.allclose(pos, (0.2763, 0.0, 0.1607), atol=0.002), pos
    assert abs(pos[1]) < 1e-6, pos


def test_the_readme_torque_off_table_has_the_urdf_signs():
    """Each row of the README table: +0.05 rad from the folded rest moves the arm that way."""
    robot = robot_of(render())

    def rise(joint, link):  # the height change of link (mount_link is z up)
        return fk(robot, link, {**REST, joint: REST[joint] + 0.05})[2, 3] - fk(robot, link, REST)[2, 3]

    def axis(joint):  # in mount_link. Seen from where it points, + turns counterclockwise.
        j = robot.joint_map[joint]
        return fk(robot, j.child, REST)[:3, :3] @ j.axis

    assert axis("link_002_joint")[2] < -0.99  # + is clockwise as seen from above
    assert rise("link_003_joint", "link_004") < 0  # + lowers the elbow, so a lift is -
    assert rise("link_004_joint", "link_005") > 0  # + lifts the forearm off the upper arm
    assert rise("link_005_joint", "grasp_center") > 0  # + raises the jaw tips
    assert axis("link_006_joint")[2] < -0.8  # + is clockwise as seen from above
    wrist_to_tips = fk(robot, "grasp_center", REST)[:3, 3] - fk(robot, "link_007", REST)[:3, 3]
    wrist_to_tips /= np.linalg.norm(wrist_to_tips)
    assert axis("link_007_joint") @ wrist_to_tips < -0.99  # + is clockwise as seen from the tips
    assert robot.joint_map["gripper_left_joint"].limit.lower == 0.0  # 0 is closed, + opens


def test_every_mesh_uri_resolves_to_a_file():
    doc = render()
    uris = [m.getAttribute("filename") for m in doc.getElementsByTagName("mesh")]
    assert uris
    prefix = "package://maker_arm_description/"
    for uri in uris:
        assert uri.startswith(prefix), uri
        assert (PKG / uri[len(prefix) :]).is_file(), uri


def test_the_desk_top_is_just_below_the_mount_plane():
    robot = robot_of(render())
    assert robot.joint_map["desk_joint"].type == "fixed"
    assert robot.joint_map["desk_joint"].parent == "mount_link"
    col = robot.link_map["desk_link"].collision
    assert np.allclose(col.geometry.size, (1.2, 1.2, 0.02))
    top = fk(robot, "desk_link", {})[2, 3] + col.origin.xyz[2] + col.geometry.size[2] / 2
    assert abs(top - -0.001) < 1e-9
    assert "desk_link" not in robot_of(render(desk="false")).link_map


def test_the_ros2_control_block_has_the_seven_joints_and_the_parameters():
    doc = render(interface="can", rt="true", socket="/tmp/s.sock")
    (rc,) = doc.getElementsByTagName("ros2_control")
    assert rc.getAttribute("name") == "MakerArm"
    (plugin,) = rc.getElementsByTagName("plugin")
    assert plugin.firstChild.data == "maker_arm_hardware/MakerArmSystem"
    params = {
        p.getAttribute("name"): (p.firstChild.data if p.firstChild else "")
        for p in rc.getElementsByTagName("param")
    }
    assert params["interface"] == "can" and params["rt"] == "true"
    assert params["socket"] == "/tmp/s.sock"
    assert params["executable"] == "maker-arm-cli" and params["persist"] == "false"
    assert params["profile"].endswith("config/daemon/sim_profile.json")
    assert params["dynamics"].endswith("config/daemon/dynamics_urdf.json")
    for name in ("params", "residual", "gripper_map", "log"):
        assert params[name] == ""
    joints = rc.getElementsByTagName("joint")
    assert [j.getAttribute("name") for j in joints] == ARM_AND_GRIPPER
    for j in joints:
        cmds = [c.getAttribute("name") for c in j.getElementsByTagName("command_interface")]
        states = [s.getAttribute("name") for s in j.getElementsByTagName("state_interface")]
        assert cmds == ["position"] and states == ["position", "velocity", "effort"]
    assert not robot_of(render(use_ros2_control="false")).transmissions
    assert not render(use_ros2_control="false").getElementsByTagName("ros2_control")


def test_the_daemon_files_are_the_cli_exports():
    d = PKG / "config" / "daemon"
    profile = json.loads((d / "sim_profile.json").read_text())
    assert {"joints", "control_rate_hz", "max_velocity", "limit_margin"} <= set(profile)
    assert len(profile["joints"]) == 7
    dyn = json.loads((d / "dynamics_urdf.json").read_text())
    assert {"mount_rpy", "links", "gravity", "bodies", "friction", "armature", "provenance"} == set(dyn)
    assert dyn["provenance"]["source"] == "urdf" and len(dyn["provenance"]["urdf_sha256"]) == 64
    assert len(dyn["links"]) == 6 and len(dyn["bodies"]) >= 6
    assert np.allclose(dyn["mount_rpy"], (math.pi / 2, 0, math.pi))


def test_the_real_profiles_are_the_cli_exports(tmp_path):
    cal = DAEMON / "vendor_calibration.json"
    for caps, name in (("0.5", "real_profile_caps05.json"), ("1.0", "real_profile_nominal.json")):
        out = tmp_path / name
        subprocess.run(["maker-arm-cli", "export", "--calibration", str(cal), "--caps", caps,
                        "--out", str(out)], check=True)
        assert out.read_bytes() == (DAEMON / name).read_bytes(), name


def test_the_vendor_calibration_is_the_gravity_convention():
    cal = json.loads((DAEMON / "vendor_calibration.json").read_text())
    assert cal == {"direction": [1, 1, 1, 1, -1, 1, 1], "offset": [0] * 7}
