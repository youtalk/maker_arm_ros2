"""The torque-off check: live joint states in RViz. No motor is enabled and no daemon starts."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DAEMON = os.path.join(share("maker_arm_description"), "config", "daemon")


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ("interface", "profile", "gripper_map")}
    xacro = os.path.join(share("maker_arm_description"), "urdf", "maker_arm.urdf.xacro")
    description = {"robot_description": ParameterValue(
        Command(["xacro ", xacro, " use_ros2_control:=false"]), value_type=str)}
    rviz_cfg = os.path.join(share("maker_arm_bringup"), "rviz", "torque_off.rviz")
    defaults = {"interface": "can0", "profile": os.path.join(DAEMON, "real_profile_caps05.json"),
                "gripper_map": os.path.join(DAEMON, "gripper_map.txt")}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             respawn=True, respawn_delay=2.0),
        Node(package="maker_arm_bringup", executable="torque_off_view", output="screen",
             arguments=["--interface", cfg["interface"], "--profile", cfg["profile"],
                        "--gripper-map", cfg["gripper_map"]]),
    ])
