import importlib.machinery
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

PKG = Path(__file__).resolve().parents[1]
GO_HOME = PKG / "scripts" / "go_home"
ARM = [f"link_00{i}_joint" for i in range(2, 8)]
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
    names = ["link_003_joint", "link_002_joint", "link_004_joint"]
    assert list(arm.trajectory.joint_names) == names
    (point,) = arm.trajectory.points
    assert list(point.positions) == [-0.15, 0.1, 0.14]
    assert (point.time_from_start.sec, point.time_from_start.nanosec) == (5, 0)
    assert hand.command.position == 0.003


def test_a_missing_state_is_named():
    with pytest.raises(ValueError, match="arm state home"):
        load().goals('<robot name="maker_arm"/>')


def test_go_home_moves_only_from_near_home():
    go_home = load()
    srdf = (PKG.parent / "maker_arm_moveit_config" / "config" / "maker_arm.srdf").read_text()
    home = go_home.state(srdf, "arm", "home")
    rest = (-0.14, -0.056, 0.049, -0.43, 0.045, -0.056)  # a folded rest after enable
    go_home.check_start(home, dict(zip(ARM, rest)))
    work = (0.0, -0.95, 1.2, -0.7, -0.1, 0.0)
    refused = r"link_003_joint is 0\.794 rad from home \(limit 0\.6\)"
    with pytest.raises(RuntimeError, match=refused):
        go_home.check_start(home, dict(zip(ARM, work)))
    with pytest.raises(RuntimeError, match="link_007_joint is nan"):  # a joint state is missing
        go_home.check_start(home, dict(zip(ARM[:5], rest)))


def test_go_home_checks_that_the_arm_arrived():
    go_home = load()
    home = {"link_002_joint": 0.0, "link_003_joint": -0.1}
    closed = {"gripper_left_joint": 0.003}
    near = {"link_002_joint": 0.0201, "link_003_joint": -0.1, "gripper_left_joint": 0.0049}
    go_home.check_arrival(home, closed, near)  # J4 at caps 0.5 held 20.1 mrad off home
    with pytest.raises(RuntimeError, match=r"link_003_joint stopped 0\.0310 rad .*limit 0\.03 rad"):
        go_home.check_arrival(home, closed, {**near, "link_003_joint": -0.131})
    go_home.check_arrival(home, closed, {**near, "gripper_left_joint": 0.0014})  # past the goal
    with pytest.raises(RuntimeError, match=r"gripper_left_joint stopped 0\.0021 m .*limit 0\.002 m"):
        go_home.check_arrival(home, closed, {**near, "gripper_left_joint": 0.0051})
    with pytest.raises(RuntimeError, match="gripper_left_joint stopped nan"):
        go_home.check_arrival(home, closed, {"link_002_joint": 0.0, "link_003_joint": -0.1})


class FakeGoal:
    """The goal handle of an accepted goal. Its futures are names that the fake wait answers."""

    accepted = True
    cancelled = False

    def get_result_async(self):
        return "result"

    def cancel_goal_async(self):
        self.cancelled = True
        return "cancel"


def test_home_is_the_same_band_as_the_park_monitors():
    loader = importlib.machinery.SourceFileLoader(
        "park_monitor", str(PKG / "scripts" / "park_monitor"))
    park_monitor = importlib.util.module_from_spec(
        importlib.util.spec_from_loader("park_monitor", loader))
    loader.exec_module(park_monitor)
    assert load().ARRIVED_RAD == park_monitor.NEAR_RAD == 0.03  # ruling R37


@pytest.mark.parametrize("stop", [KeyboardInterrupt(), RuntimeError("no result in 15.0 s")])
def test_ctrl_c_or_a_timeout_during_the_move_cancels_the_goal(monkeypatch, capsys, stop):
    go_home = load()
    goal = FakeGoal()

    class Client:
        def __init__(self, node, action, name):
            pass

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, g):
            return "sent"

    def wait(node, future, timeout, what):
        if future == "result":
            raise stop
        if future == "cancel":
            return SimpleNamespace(goals_canceling=[object()], return_code=0)
        return goal

    monkeypatch.setattr(go_home, "ActionClient", Client)
    monkeypatch.setattr(go_home, "wait", wait)
    with pytest.raises(type(stop)):
        go_home.send(None, None, "/arm", None, lambda r: True)
    assert goal.cancelled
    assert "cancelled the /arm goal" in capsys.readouterr().err


class FakeClient:
    """A pause_servo client: up or not, and the data of each call."""

    def __init__(self, up):
        self.up, self.calls = up, []

    def wait_for_service(self, timeout_sec):
        return self.up

    def call_async(self, request):
        self.calls.append(request.data)
        return "reply"


def fake_node(client):
    return SimpleNamespace(create_client=lambda srv, name: client)


def test_without_servo_go_home_goes_on(capsys):
    client = FakeClient(up=False)
    load().pause_servo(fake_node(client))
    assert client.calls == []
    assert "no /servo_node/pause_servo service" in capsys.readouterr().out


@pytest.mark.parametrize("reply", [
    SimpleNamespace(success=False, message="no"),
    RuntimeError("no /servo_node/pause_servo reply in 2.0 s"),
])
def test_a_failed_or_late_pause_stops_go_home(monkeypatch, reply):
    go_home = load()
    client = FakeClient(up=True)

    def wait(node, future, timeout, what):
        if isinstance(reply, Exception):
            raise reply
        return reply

    monkeypatch.setattr(go_home, "wait", wait)
    with pytest.raises(RuntimeError, match="pause"):
        go_home.pause_servo(fake_node(client))
    assert client.calls == [True]


def run_main(monkeypatch, pause):
    """Run main() with the ROS calls faked. Return its exit code and the calls in order."""
    go_home = load()
    srdf = (PKG.parent / "maker_arm_moveit_config" / "config" / "maker_arm.srdf").read_text()
    at_home = {**go_home.state(srdf, "arm", "home"), **go_home.state(srdf, "gripper", "closed")}
    calls = []
    monkeypatch.setattr(go_home, "semantic", lambda node: srdf)
    monkeypatch.setattr(go_home, "joint_states", lambda node: at_home)
    monkeypatch.setattr(go_home, "pause_servo", lambda node: calls.append("pause") or pause())
    monkeypatch.setattr(go_home, "send", lambda node, action, name, goal, ok: calls.append(name))
    monkeypatch.setattr(go_home, "SETTLE_S", 0.0)
    return go_home.main(), calls


def test_go_home_pauses_servo_before_the_arm_goal(monkeypatch):
    code, calls = run_main(monkeypatch, lambda: None)
    assert code == 0
    assert calls == ["pause", "/arm_controller/follow_joint_trajectory",
                     "/gripper_controller/gripper_cmd"]


def test_a_failed_pause_sends_no_goal(monkeypatch):
    def fail():
        raise RuntimeError("did not pause Servo")

    code, calls = run_main(monkeypatch, fail)
    assert code == 1 and calls == ["pause"]
