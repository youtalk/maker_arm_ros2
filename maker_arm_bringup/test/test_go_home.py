import importlib.machinery
import importlib.util
from pathlib import Path

import pytest

GO_HOME = Path(__file__).resolve().parents[1] / "scripts" / "go_home"
# The joints out of name order, and other states around home and closed.
SRDF = """<robot name="maker_arm">
    <group_state name="ready" group="arm">
        <joint name="link_002_joint" value="9"/>
    </group_state>
    <group_state name="home" group="arm">
        <joint name="link_003_joint" value="-0.15"/>
        <joint name="link_002_joint" value="0.1"/>
        <joint name="link_004_joint" value="0.14"/>
    </group_state>
    <group_state name="open" group="gripper">
        <joint name="gripper_left_joint" value="0.04"/>
    </group_state>
    <group_state name="closed" group="gripper">
        <joint name="gripper_left_joint" value="0.003"/>
    </group_state>
</robot>"""


def load():
    loader = importlib.machinery.SourceFileLoader("go_home", str(GO_HOME))
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader("go_home", loader))
    loader.exec_module(module)
    return module


def test_the_goals_are_home_in_5_s_and_the_closed_jaw():
    arm, hand = load().goals(SRDF)
    assert list(arm.trajectory.joint_names) == ["link_003_joint", "link_002_joint", "link_004_joint"]
    (point,) = arm.trajectory.points
    assert list(point.positions) == [-0.15, 0.1, 0.14]
    assert (point.time_from_start.sec, point.time_from_start.nanosec) == (5, 0)
    assert hand.command.position == 0.003


def test_a_missing_state_is_named():
    with pytest.raises(ValueError, match="arm state home"):
        load().goals('<robot name="maker_arm"/>')
