import json
import subprocess
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
DESC = PKG.parent / "maker_arm_description"
URDF = DESC / "urdf" / "maker_arm_sdk.urdf"
PROFILES = sorted(DESC.glob("config/daemon/*profile*.json"))
SCRIPT = PKG / "scripts" / "limits_from_profiles.py"
MARGIN = 0.05
ARM = [f"link_00{i}_joint" for i in range(2, 8)]
JAW_HALF_STROKE = 0.0524125  # m, the URDF's jaw limit
# maker-arm-cli's GripMap::provisional, the vendor's preview: jaw gap = A * angle + B. The jaw
# closes toward the gripper's q_hi.
A, B = -0.0502590, 0.000337389


def moveit_limits():
    cfg = yaml.safe_load((PKG / "config" / "joint_limits.yaml").read_text())
    return {j: (v["min_position"], v["max_position"]) for j, v in cfg["joint_limits"].items()}


def run_script(out):
    args = [sys.executable, str(SCRIPT), "--urdf", str(URDF), "--margin", str(MARGIN)]
    for p in PROFILES:
        args += ["--profile", str(p)]
    subprocess.run([*args, "--out", str(out)], check=True)


def test_every_moveit_limit_is_inside_every_profile_and_the_urdf():
    assert PROFILES
    limits = moveit_limits()
    assert list(limits)[:6] == ARM
    for prof in PROFILES:
        joints = json.loads(prof.read_text())["joints"]
        for i, name in enumerate(ARM):
            lo, hi = limits[name]
            # 5e-8: the yaml prints 7 decimals
            assert lo >= joints[i]["q_lo"] + MARGIN - 5e-8
            assert hi <= joints[i]["q_hi"] - MARGIN + 5e-8


def test_the_gripper_range_uses_the_vendor_map():
    lo, hi = moveit_limits()["gripper_left_joint"]
    want_lo, want_hi = 0.0, JAW_HALF_STROKE  # the URDF range
    for prof in PROFILES:
        g = json.loads(prof.read_text())["joints"][6]
        # gripper_left_joint is half the gap: the closed side from q_hi, the open side from q_lo.
        want_lo = max(want_lo, (A * (g["q_hi"] - MARGIN) + B) / 2)
        want_hi = min(want_hi, (A * (g["q_lo"] + MARGIN) + B) / 2)
    # 1e-7: the yaml prints 7 decimals, and A and B have 6 digits
    assert abs(lo - want_lo) < 1e-7 and abs(hi - want_hi) < 1e-7, (lo, hi, want_lo, want_hi)


def test_the_yaml_is_what_the_script_writes(tmp_path):
    run_script(tmp_path / "a.yaml")
    run_script(tmp_path / "b.yaml")
    assert (tmp_path / "a.yaml").read_bytes() == (tmp_path / "b.yaml").read_bytes()
    assert (tmp_path / "a.yaml").read_bytes() == (PKG / "config" / "joint_limits.yaml").read_bytes()


def test_velocity_and_acceleration_limits():
    cfg = yaml.safe_load((PKG / "config" / "joint_limits.yaml").read_text())
    assert cfg["default_velocity_scaling_factor"] == 0.2
    assert cfg["default_acceleration_scaling_factor"] == 0.2
    for name in ARM:
        j = cfg["joint_limits"][name]
        assert j["has_velocity_limits"] and j["max_velocity"] == 2.0
        assert j["has_acceleration_limits"] and j["max_acceleration"] == 4.0
    assert cfg["joint_limits"]["gripper_left_joint"]["max_velocity"] == 0.05


def test_servo_yaml_uses_only_parameters_the_installed_servo_defines():
    from ament_index_python.packages import get_package_share_directory

    defs = (Path(get_package_share_directory("moveit_servo")) / "config" / "servo_parameters.yaml")
    text = defs.read_text()
    servo = yaml.safe_load((PKG / "config" / "servo.yaml").read_text())
    for key in servo:
        assert f"\n  {key}: " in text, key
    assert servo["move_group_name"] == "arm"
    assert servo["command_out_topic"] == "/arm_controller/joint_trajectory"
