"""The stack on the real arm through the daemon."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

DAEMON = os.path.join(share("maker_arm_description"), "config", "daemon")
EXPORT = "maker-arm-cli export --calibration CAL.json --caps 0.5 --out <path>"
# The file arguments. An empty value is skipped and the daemon applies its own default.
FILES = ("profile", "dynamics", "params", "gripper_map")


def camera_nodes(context):
    """One v4l2_camera node for each camera argument that is not empty. 640 x 480 YUYV, 30 fps."""
    return [Node(package="v4l2_camera", executable="v4l2_camera_node", namespace=cam,
                 parameters=[{"video_device": dev, "image_size": [640, 480]}])
            for cam in ("wrist_camera", "head_camera")
            if (dev := context.launch_configurations[cam])]


def launch_setup(context):
    names = [*FILES, "residual", "interface", "rt", "persist", "rviz", "auto_park"]
    args = {k: context.launch_configurations[k] for k in names}
    if not args["profile"]:
        raise RuntimeError(
            "real.launch.py needs a profile. The default real_profile_caps05.json is for an arm "
            "in the vendor zero. For an arm with another zero, make one with: " + EXPORT
            + " Then pass profile:=<path>.")
    missing = [f"{k} ({v})" for k, v in args.items() if k in FILES and v and not os.path.isfile(v)]
    if missing:
        raise RuntimeError("real.launch.py needs files that do not exist: " + ", ".join(missing))
    if args["gripper_map"]:
        with open(args["gripper_map"]) as f:
            args["gripper_map"] = f.read().strip()  # one line "a,b"
    stack = PythonLaunchDescriptionSource(
        os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py"))
    return [IncludeLaunchDescription(stack, launch_arguments=args.items()), *camera_nodes(context)]


def generate_launch_description():
    # No residual until MA3 ships one. The plugin names a residual file it cannot read.
    # The shipped profile is the vendor calibration at caps 0.5.
    defaults = {"profile": os.path.join(DAEMON, "real_profile_caps05.json"),
                "dynamics": os.path.join(DAEMON, "dynamics_urdf.json"),
                "params": "", "gripper_map": os.path.join(DAEMON, "gripper_map.txt"), "interface": "can0", "residual": "",
                "rt": "false", "persist": "false", "rviz": "false", "auto_park": "true",
                "wrist_camera": "", "head_camera": ""}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=launch_setup)])
