import os
import subprocess
from pathlib import Path

WAKE = Path(__file__).resolve().parents[1] / "scripts" / "wake"


def run(tmp_path, *args, enable_exit=0):
    """Run wake with a fake maker-arm-cli and ros2 that log their arguments."""
    log = tmp_path / "calls"
    log.unlink(missing_ok=True)
    for exe, code in (("maker-arm-cli", enable_exit), ("ros2", 0)):
        fake = tmp_path / exe
        fake.write_text(f'#!/usr/bin/env bash\necho "{exe} $@" >> {log}\nexit {code}\n')
        fake.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"}
    r = subprocess.run([str(WAKE), *args], env=env, capture_output=True, text=True)
    return r, log.read_text().splitlines()


def test_wake_enables_then_restarts_the_controllers_at_the_measured_pose_then_goes_home(tmp_path):
    r, calls = run(tmp_path)
    assert r.returncode == 0, r.stderr
    assert calls == [
        "maker-arm-cli enable --interface can0",
        "ros2 control switch_controllers --strict --deactivate arm_controller gripper_controller",
        "ros2 control set_hardware_component_state MakerArm inactive",
        "ros2 control set_hardware_component_state MakerArm active",
        "ros2 control switch_controllers --strict --activate arm_controller gripper_controller",
        "ros2 run maker_arm_bringup go_home",
    ]


def test_wake_keeps_the_callers_interface(tmp_path):
    assert run(tmp_path, "--interface", "vcan0")[1][0] == "maker-arm-cli enable --interface vcan0"
    assert run(tmp_path, "--interface=can1", "--socket", "/s")[1][0] == (
        "maker-arm-cli enable --interface=can1 --socket /s")


def test_a_refused_enable_restarts_nothing(tmp_path):
    r, calls = run(tmp_path, enable_exit=1)
    assert r.returncode == 1
    assert calls == ["maker-arm-cli enable --interface can0"]
    assert "Torque stays off" in r.stderr
