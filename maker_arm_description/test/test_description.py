import json
import math
import os
from pathlib import Path

import numpy as np
import xacro
from urdf_parser_py.urdf import URDF

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / "urdf" / "maker_arm.urdf.xacro"
HOME = {
    "link_002_joint": 0.0,
    "link_003_joint": -1.79,
    "link_004_joint": 2.05,
    "link_005_joint": -1.3,
    "link_006_joint": -0.22,
    "link_007_joint": 0.0,
}
ARM_AND_GRIPPER = [*HOME, "gripper_left_joint"]


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


def test_the_ready_pose_puts_grasp_center_where_the_lab_measured_it():
    robot = robot_of(render())
    pos = fk(robot, "grasp_center", HOME)[:3, 3]
    assert np.allclose(pos, (0.2749, 0.0371, 0.1646), atol=0.002), pos


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


def test_the_daemon_files_are_the_labs_exports():
    d = PKG / "config" / "daemon"
    profile = json.loads((d / "sim_profile.json").read_text())
    assert {"joints", "control_rate_hz", "max_velocity", "limit_margin"} <= set(profile)
    assert len(profile["joints"]) == 7
    dyn = json.loads((d / "dynamics_urdf.json").read_text())
    assert {"mount_rpy", "links", "gravity", "bodies", "friction", "armature", "provenance"} == set(dyn)
    assert dyn["provenance"]["source"] == "urdf"
    assert len(dyn["links"]) == 6 and len(dyn["bodies"]) >= 6
    assert np.allclose(dyn["mount_rpy"], (math.pi / 2, 0, math.pi))
