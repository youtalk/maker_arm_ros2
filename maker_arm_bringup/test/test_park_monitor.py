import csv
import importlib.machinery
import importlib.util
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import xacro

PKG = Path(__file__).resolve().parents[1]
SCRIPT = PKG / "scripts" / "park_monitor"
SRDF = PKG.parent / "maker_arm_moveit_config" / "config" / "maker_arm.srdf"
XACRO = PKG.parent / "maker_arm_description" / "urdf" / "maker_arm.urdf.xacro"
# /joint_states of the MuJoCo plant at 100 Hz in the sim rehearsal of Task 18: the end of a
# MoveIt move from `work` to `home` at scaling 0.2 (it starts at t = 0), then the stay at home
# up to the park. Columns: t in s, then the position of each arm joint. The live park_monitor
# logged the park at PARKED_AT_S, 3.4 s after MoveIt reported the goal done.
RECORDED = Path(__file__).resolve().parent / "fixtures" / "moveit_home_joint_states.csv"
PARKED_AT_S = 7.158
ARM = [f"link_00{i}_joint" for i in range(2, 8)]


def load():
    loader = importlib.machinery.SourceFileLoader("park_monitor", str(SCRIPT))
    spec = importlib.util.spec_from_loader("park_monitor", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def home():
    return load().home_of(SRDF.read_text())


def near(offset=0.0, joint="link_004_joint"):
    """Return home with `joint` moved by `offset`."""
    h = home()
    return {**h, joint: h[joint] + offset}


def run(rule, stream):
    """Feed (t, position, busy) samples. Return the times at which the rule fired."""
    return [t for t, p, busy in stream if rule.step(t, p, busy)]


def quarters(n, position=None, busy=lambda k: False, start=0):
    """Samples every 0.25 s, exact in binary, from start / 4 s."""
    return [(k / 4, position or near(), busy(k)) for k in range(start, start + n)]


def armed(pm):
    """A rule whose last enable was ARM_S ago, so a stay at home parks."""
    return pm.Rule(home(), -pm.ARM_S)


def test_a_still_arm_at_home_parks_once_3_s_after_the_first_speed_window():
    pm = load()
    assert run(armed(pm), quarters(40)) == [pm.WINDOW_S + pm.HOLD_S]


@pytest.mark.parametrize("speed, parks", [(0.009, True), (0.011, False)])
def test_a_joint_parks_only_below_0_01_rad_s(speed, parks):
    pm = load()
    stream = [(k / 4, near(-0.02 + speed * k / 4), False) for k in range(19)]  # 4.5 s near home
    assert bool(run(armed(pm), stream)) == parks


def test_a_joint_off_home_or_a_busy_controller_blocks_the_park():
    pm = load()
    assert run(armed(pm), quarters(40, near(0.031))) == []
    assert run(armed(pm), quarters(40, busy=lambda k: True)) == []


def test_a_break_restarts_the_3_s_and_allows_one_more_park():
    pm = load()
    stream = quarters(24, busy=lambda k: k == 8)  # a twist or a goal at 2.0 s
    stream += [(6.0, near(), False), (6.25, near(0.1), False)]  # parked, then a move away
    stream += quarters(20, start=26)
    assert run(armed(pm), stream) == [5.25, 6.5 + pm.HOLD_S]


def test_a_missing_joint_never_parks():
    pm = load()
    p = near()
    del p["link_007_joint"]
    assert run(armed(pm), quarters(40, p)) == []


def test_after_an_enable_go_home_from_the_rest_parks_only_after_60_s():
    pm = load()
    rest = near(-0.3, "link_003_joint")
    stream = quarters(8, rest) + quarters(400, start=8)  # go_home at 2 s, then 100 s at home
    assert run(pm.Rule(home(), 0.0), stream) == [pm.ARM_S]


def test_after_an_enable_a_move_away_and_back_arms_the_park():
    pm = load()
    stream = quarters(40)  # 10 s at home: no park
    stream += quarters(8, near(0.2), start=40)  # MoveIt to work
    stream += quarters(40, start=48)  # back at home at 12 s
    assert run(pm.Rule(home(), 0.0), stream) == [12.5 + pm.HOLD_S]


def test_only_a_move_past_0_15_rad_arms_the_park():
    pm = load()
    for offset, fired in ((0.1, [pm.ARM_S]), (0.16, [10.0 + pm.WINDOW_S + pm.HOLD_S])):
        stream = quarters(32)  # 8 s at home
        stream += quarters(8, near(offset), start=32)  # a move of `offset` for 2 s
        stream += quarters(240, start=40)  # back at home at 10 s
        assert run(pm.Rule(home(), 0.0), stream) == fired


def test_a_new_enable_disarms_the_park_again():
    pm = load()
    rule = armed(pm)
    assert run(rule, quarters(8, near(0.2))) == []  # away, so armed
    rule.enabled(2.0)  # wake: the arm drops onto the rest, then the daemon enables it
    assert run(rule, quarters(200, start=8)) == []  # go_home, then 48 s at home
    assert run(rule, quarters(60, start=208)) == [62.0]


def test_a_gap_in_the_joint_states_restarts_the_3_s():
    pm = load()
    stream = quarters(8) + quarters(40, start=14)  # no sample from 1.75 s to 3.5 s
    assert run(armed(pm), stream) == [4.0 + pm.HOLD_S]


def test_a_failed_park_tries_again_every_5_s():
    pm = load()
    rule = armed(pm)
    fired = []
    for t, p, busy in quarters(80):
        if rule.step(t, p, busy):
            fired.append(t)
            rule.failed(t)
    assert fired == [3.5, 8.5, 13.5, 18.5]


@pytest.mark.parametrize("value, moves", [(0.0, False), (1e-7, False), (-2e-6, True)])
def test_only_a_non_zero_twist_is_servo_activity(value, moves):
    from geometry_msgs.msg import Twist
    for axis in ("linear", "angular"):
        t = Twist()
        setattr(getattr(t, axis), "z", value)
        assert load().moving(t) == moves


def test_an_enable_is_a_change_from_released_to_holding_or_following():
    pm = load()
    assert pm.enabled_again("released", "holding") and pm.enabled_again("released", "following")
    assert not pm.enabled_again(None, "holding")  # the monitor's start counts already
    assert not pm.enabled_again("refused", "following")
    assert not pm.enabled_again("released", None)  # no answer


def test_the_recorded_moveit_home_parks_when_the_live_monitor_did():
    pm = load()
    with RECORDED.open() as f:
        stream = [(t, dict(zip(ARM, q)), False)
                  for t, *q in (map(float, row) for row in csv.reader(f))]
    (fired,) = run(armed(pm), stream)
    arrived = max(t for t, p, _ in stream if any(
        abs(p[j] - q) > pm.NEAR_RAD for j, q in home().items()))
    assert arrived + pm.HOLD_S < fired
    # The replay has no goal status, and its speed windows start at other samples.
    assert abs(fired - PARKED_AT_S) <= pm.WINDOW_S


def test_the_daemon_command_comes_from_the_plugin_parameters():
    pm = load()
    urdf = xacro.process_file(str(XACRO), mappings={"interface": "vcan0"}).toxml()
    assert pm.daemon_cli(urdf) == ["maker-arm-cli", "--interface", "vcan0"]
    urdf = xacro.process_file(str(XACRO), mappings={
        "interface": "can1", "executable": "/opt/cli", "socket": "/run/s.sock"}).toxml()
    assert pm.daemon_cli(urdf) == ["/opt/cli", "--interface", "can1", "--socket", "/run/s.sock"]
    with pytest.raises(ValueError, match="interface"):
        pm.daemon_cli('<robot name="r"/>')


def fake_cli(tmp_path, status):
    """A maker-arm-cli that logs its arguments and prints `status` for `status`."""
    log = tmp_path / "calls"
    cli = tmp_path / "maker-arm-cli"
    cli.write_text(f'#!/usr/bin/env bash\necho "$@" >> {log}\n'
                   f'[ "$1" = status ] && printf \'%s\\n\' \'{status}\'\nexit 0\n')
    cli.chmod(0o755)
    return [str(cli), "--interface", "vcan0"], log


@pytest.mark.parametrize("mode", ["holding", "following"])
def test_park_releases_a_holding_or_following_daemon_without_the_typed_word(tmp_path, mode):
    cli, log = fake_cli(tmp_path, f'STATUS {{"mode": "{mode}", "t": 1.0}}')
    assert load().park(cli) == (True, False, "parked: torque off at home")
    assert log.read_text().splitlines() == [
        "status --interface vcan0", "release --interface vcan0 --yes"]


@pytest.mark.parametrize("status, retry", [
    ('STATUS {"mode": "released"}', False), ('STATUS {"mode": "fault"}', True),
    ('STATUS {"mode": "refused"}', True), ("", True), ("STATUS [1]", True)])
def test_park_never_releases_another_mode_or_a_silent_daemon(tmp_path, status, retry):
    cli, log = fake_cli(tmp_path, status)
    released, again, line = load().park(cli)
    assert not released and again == retry and line.startswith("not parking")
    assert log.read_text().splitlines() == ["status --interface vcan0"]


def test_a_failed_release_is_no_park(tmp_path):
    cli, log = fake_cli(tmp_path, 'STATUS {"mode": "following"}')
    Path(cli[0]).write_text(Path(cli[0]).read_text().replace(
        "exit 0", '[ "$1" = release ] && echo "release failed: X" >&2 && exit 1\nexit 0'))
    assert load().park(cli) == (False, True, "park failed: release failed: X")


def idle(stream, limit=4.0, mode="holding"):
    """Feed (t, position, still, busy) samples to idle_return. Return the times it was due."""
    pm, h, since, due_at = load(), home(), None, []
    for t, p, still, busy in stream:
        since, due = pm.idle_return(since, t, limit, h, p, mode, still, busy)
        if due:
            due_at.append(t)
    return due_at


def stays(n, position=None, still=True, busy=lambda k: False):
    """Samples every 0.25 s from 0 s, by default 0.031 rad from home: just outside its band."""
    return [(k / 4, position or near(0.031), still, busy(k)) for k in range(n)]


def test_an_idle_time_below_the_limit_gives_no_return():
    assert idle(stays(16)) == []  # 3.75 s of 4 s


def test_a_still_arm_away_from_home_returns_after_the_limit_and_then_counts_again():
    assert idle(stays(40)) == [4.0, 8.25]


@pytest.mark.parametrize("still, busy", [(True, True), (False, False)])
def test_a_twist_a_goal_or_a_move_restarts_the_count(still, busy):
    stream = stays(28)
    stream[8] = (2.0, stream[8][1], still, busy)
    assert idle(stream) == [6.25]


@pytest.mark.parametrize("mode", ["released", "fault", "refused", None])
def test_a_daemon_that_is_not_holding_or_following_gives_no_return(mode):
    assert idle(stays(40), mode=mode) == []
    assert idle(stays(40), mode="following") == [4.0, 8.25]


def test_at_home_the_park_acts_and_not_the_return():
    assert idle(stays(40, near(0.03))) == []


def test_idle_home_s_0_turns_the_return_off():
    assert idle(stays(40), limit=0.0) == []


def test_the_return_is_one_pilz_ptp_goal_to_the_srdf_home():
    r = load().move_goal(home()).request
    assert (r.group_name, r.pipeline_id, r.planner_id) == (
        "arm", "pilz_industrial_motion_planner", "PTP")
    assert (r.max_velocity_scaling_factor, r.max_acceleration_scaling_factor) == (0.1, 0.1)
    (goal,) = r.goal_constraints
    assert {c.joint_name: c.position for c in goal.joint_constraints} == home()
    assert {(c.tolerance_above, c.tolerance_below) for c in goal.joint_constraints} == {
        (0.01, 0.01)}


def test_the_servo_pause_is_go_homes():
    loader = importlib.machinery.SourceFileLoader("go_home", str(PKG / "scripts" / "go_home"))
    go_home = importlib.util.module_from_spec(importlib.util.spec_from_loader("go_home", loader))
    loader.exec_module(go_home)
    pm = load()
    assert (pm.SERVO_PAUSE, pm.PAUSE_WAIT_S) == (go_home.SERVO_PAUSE, go_home.PAUSE_WAIT_S)


class Done:
    """A future that is done, with a result."""

    def __init__(self, result):
        self.value = result

    def done(self):
        return True

    def result(self):
        return self.value


class Never:
    """A future that never completes."""

    def done(self):
        return False


class FakePause:
    def __init__(self, up, reply=None):
        self.up, self.reply, self.calls = up, reply, 0

    def wait_for_service(self, timeout_sec):
        return self.up

    def call_async(self, request):
        assert request.data
        self.calls += 1
        return Done(self.reply) if self.reply else Never()


class FakeMove:
    """A MoveGroup client whose goal is accepted and then runs until it is cancelled. A twist
    arrives once the goal is out."""

    def __init__(self, stop):
        self.stop, self.sent, self.cancelled = stop, [], False

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        self.stop.set()
        move = self

        class Handle:
            accepted = True

            def get_result_async(self):
                return Never()

            def cancel_goal_async(self):
                move.cancelled = True

        return Done(Handle())


def test_a_twist_during_the_return_cancels_the_goal():
    pm = load()
    stop = threading.Event()
    move = FakeMove(stop)
    # Without Servo the goal goes out.
    assert pm.return_home(FakePause(up=False), move, "goal", stop) == (
        False, "return home cancelled: a twist arrived")
    assert move.sent == ["goal"] and move.cancelled


def test_a_refused_or_late_servo_pause_sends_no_goal(monkeypatch):
    pm = load()
    monkeypatch.setattr(pm, "PAUSE_WAIT_S", 0.1)
    stop = threading.Event()
    move = FakeMove(stop)
    refused = FakePause(up=True, reply=SimpleNamespace(success=False, message="no"))
    for pause in (refused, FakePause(up=True)):
        returned, line = pm.return_home(pause, move, "goal", stop)
        assert not returned and line.startswith("not returning home") and pause.calls == 1
    assert move.sent == []
