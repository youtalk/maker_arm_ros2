#!/usr/bin/env python3
"""Write MoveIt's joint_limits.yaml from the URDF limits and the daemon profiles.

Arm joint i of a profile is URDF joint link_00{i+2}_joint. The position range of each arm
joint is the intersection of the URDF range and every profile's [q_lo + margin, q_hi - margin].
The gripper gets the same margin in motor radians, mapped to metres through maker-arm-cli's
provisional grip map: a jaw gap of 0 at the gripper's q_lo and 2 * JAW_HALF_STROKE at its q_hi,
and gripper_left_joint is half the gap. That range is intersected with the URDF range too.
"""
import argparse
import json
import xml.etree.ElementTree as ET

ARM = [f"link_00{i}_joint" for i in range(2, 8)]
GRIPPER = "gripper_left_joint"
JAW_HALF_STROKE = 0.0524125  # m, as in maker-arm-cli's provisional grip map (files.rs)


def urdf_limits(path):
    out = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        lim = joint.find("limit")
        if lim is not None:
            out[joint.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def arm_ranges(urdf, profiles, margin):
    ranges = {}
    for i, name in enumerate(ARM):
        lo, hi = urdf[name]
        for profile in profiles:
            joint = profile["joints"][i]
            lo = max(lo, joint["q_lo"] + margin)
            hi = min(hi, joint["q_hi"] - margin)
        assert lo < hi, f"{name}: empty range"
        ranges[name] = (lo, hi)
    return ranges


def gripper_range(urdf, profiles, margin):
    lo, hi = urdf[GRIPPER]
    for profile in profiles:
        g = profile["joints"][len(ARM)]
        m = margin * JAW_HALF_STROKE / (g["q_hi"] - g["q_lo"])  # the margin in metres
        lo = max(lo, m)
        hi = min(hi, JAW_HALF_STROKE - m)
    assert lo < hi, f"{GRIPPER}: empty range"
    return lo, hi


def render(urdf, profiles, margin):
    lines = [
        "# Written by scripts/limits_from_profiles.py. Do not edit by hand.",
        "default_velocity_scaling_factor: 0.2",
        "default_acceleration_scaling_factor: 0.2",
        "joint_limits:",
    ]
    ranges = arm_ranges(urdf, profiles, margin)
    ranges[GRIPPER] = gripper_range(urdf, profiles, margin)
    for name, (lo, hi) in ranges.items():
        vel, acc = ("0.05", "0.5") if name == GRIPPER else ("2.0", "4.0")
        lines += [
            f"  {name}:",
            f"    min_position: {lo:.7f}",
            f"    max_position: {hi:.7f}",
            "    has_velocity_limits: true",
            f"    max_velocity: {vel}  # INITIAL: V2 measures the real speed ceiling",
            "    has_acceleration_limits: true",
            f"    max_acceleration: {acc}  # INITIAL: V2 measures the real acceleration ceiling",
        ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--profile", action="append", required=True)
    ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    profiles = [json.load(open(p)) for p in a.profile]
    with open(a.out, "w", newline="\n") as f:
        f.write(render(urdf_limits(a.urdf), profiles, a.margin))
