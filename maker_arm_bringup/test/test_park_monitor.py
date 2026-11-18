import csv
import importlib.machinery
import importlib.util
from pathlib import Path

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
