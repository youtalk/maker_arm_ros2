import os
import subprocess
from pathlib import Path

WAKE = Path(__file__).resolve().parents[1] / "scripts" / "wake"
LISTED = "joint_state_broadcaster active\narm_controller active\ngripper_controller active"


def run(tmp_path, *args, enable_exit=0, listed=LISTED, list_exit=0):
    """Run wake with a fake maker-arm-cli and ros2 that log their arguments."""
    log = tmp_path / "calls"
    log.unlink(missing_ok=True)
    (tmp_path / "listed").write_text(listed + "\n")
    cli = tmp_path / "maker-arm-cli"
    cli.write_text(f'#!/usr/bin/env bash\necho "maker-arm-cli $@" >> {log}\nexit {enable_exit}\n')
    ros2 = tmp_path / "ros2"
    ros2.write_text(f'#!/usr/bin/env bash\necho "ros2 $@" >> {log}\n'
                    f'if [ "$2" = list_controllers ]; then cat {tmp_path / "listed"}; '
                    f'exit {list_exit}; fi\n')
    for exe in (cli, ros2):
        exe.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"}
    r = subprocess.run([str(WAKE), *args], env=env, capture_output=True, text=True)
    return r, log.read_text().splitlines()


def test_wake_enables_then_restarts_the_controllers_at_the_measured_pose_then_goes_home(tmp_path):
    r, calls = run(tmp_path)
    assert r.returncode == 0, r.stderr
    assert calls == [
        "ros2 control list_controllers",
        "maker-arm-cli enable --interface can0",
        "ros2 control switch_controllers --best-effort --deactivate arm_controller "
        "gripper_controller",
        "ros2 control set_hardware_component_state MakerArm inactive",
        "ros2 control set_hardware_component_state MakerArm active",
        "ros2 control switch_controllers --strict --activate arm_controller gripper_controller",
        "ros2 run maker_arm_bringup go_home",
    ]


def test_wake_keeps_the_callers_interface(tmp_path):
    assert run(tmp_path, "--interface", "vcan0")[1][1] == "maker-arm-cli enable --interface vcan0"
    assert run(tmp_path, "--interface=can1", "--socket", "/s")[1][1] == (
        "maker-arm-cli enable --interface=can1 --socket /s")


def test_a_refused_enable_restarts_nothing(tmp_path):
    r, calls = run(tmp_path, enable_exit=1)
    assert r.returncode == 1
    assert calls[1:] == ["maker-arm-cli enable --interface can0"]
    assert "wait about 2 s" in r.stderr


def test_without_the_controller_manager_or_a_controller_nothing_is_enabled(tmp_path):
    r, calls = run(tmp_path, list_exit=124)  # timeout ended list_controllers
    assert r.returncode == 1 and "no controller manager" in r.stderr
    assert calls == ["ros2 control list_controllers"]
    r, calls = run(tmp_path, listed="joint_state_broadcaster active\narm_controller active")
    assert r.returncode == 1 and "no gripper_controller" in r.stderr
    assert calls == ["ros2 control list_controllers"]
