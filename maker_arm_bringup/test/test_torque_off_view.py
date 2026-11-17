import importlib.machinery
import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "torque_off_view"
ARM = [f"link_00{i}_joint" for i in range(2, 8)]


def load():
    loader = importlib.machinery.SourceFileLoader("torque_off_view", str(SCRIPT))
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader("torque_off_view", loader))
    loader.exec_module(module)
    return module


def test_watch_line_becomes_a_joint_state():
    msg = load().parse_watch("WATCH 0.2 0.1 -0.2 0.3 -0.4 0.5 -0.6 0.02")
    assert list(msg.name) == ARM + ["gripper_left_joint"]
    assert list(msg.position) == [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.02]


@pytest.mark.parametrize("line", ["WATCH 0.2 0.1 -0.2", "motor 1 answers"])
def test_another_line_is_no_joint_state(line):
    assert load().parse_watch(line) is None


def test_the_joints_that_read_nan_are_named():
    view = load()
    assert view.silent(view.parse_watch("WATCH 0.2 nan -0.2 0.3 -0.4 0.5 -0.6 nan")) == [
        "link_002_joint", "gripper_left_joint"]
    assert view.silent(view.parse_watch("WATCH 0.2 0.1 -0.2 0.3 -0.4 0.5 -0.6 0.02")) == []


def test_the_command_only_scans_and_maps_the_interface():
    view = load()
    assert view.command("can0", "p.json", "") == [
        "maker-arm-cli", "--can", "can0", "scan", "--watch", "--profile", "p.json"]
    assert view.command("sim", "", "-0.03,0.01")[:2] == ["maker-arm-cli", "--sim"]
    assert view.command("sim", "", "-0.03,0.01")[-2:] == ["--gripper_map", "-0.03,0.01"]
    for iface in ("can0", "sim"):
        assert not {"serve", "hold", "enable"} & set(view.command(iface, "", ""))


def test_a_sweep_of_the_simulator_gives_joint_states():
    import subprocess
    cmd = load().command("sim", "", "") + ["--count", "2"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout.splitlines()
    msgs = [load().parse_watch(line) for line in out]
    assert len(msgs) == 2 and all(m and not any(map(math.isnan, m.position)) for m in msgs)
