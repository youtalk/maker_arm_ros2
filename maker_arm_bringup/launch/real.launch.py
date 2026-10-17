"""The stack on the real arm through the daemon, with the files MS5 produces."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

DAEMON = os.path.join(share("maker_arm_description"), "config", "daemon")
# argument -> (default file, the MS5 session that makes it)
FILES = {
    "profile": ("real_profile_caps050.json", "MS5 session 2"),
    "dynamics": ("dynamics_ident.json", "MS5 session 5"),
    "params": ("params_shipped.json", "MS5 session 5"),
    "gripper_map": ("gripper_map.txt", "MS5 session 4"),
}


def launch_setup(context):
    args = {k: context.launch_configurations[k] for k in [*FILES, "interface", "persist", "rviz"]}
    missing = [f"{v} ({FILES[k][1]})" for k, v in args.items()
               if k in FILES and not os.path.isfile(v)]
    if missing:
        raise RuntimeError("real.launch.py needs files that do not exist yet: " + ", ".join(missing))
    with open(args["gripper_map"]) as f:
        args["gripper_map"] = f.read().strip()  # one line "a,b"
    stack = PythonLaunchDescriptionSource(
        os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py"))
    return [IncludeLaunchDescription(stack, launch_arguments=args.items())]


def generate_launch_description():
    defaults = {"interface": "can0", "persist": "false", "rviz": "false"}
    defaults.update({k: os.path.join(DAEMON, v[0]) for k, v in FILES.items()})
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=launch_setup)])
