"""The stack on SimArm, the daemon's simulated arm, with the shipped simulation files."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stack = PythonLaunchDescriptionSource(
        os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py"))
    return LaunchDescription([
        DeclareLaunchArgument("persist", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(stack, launch_arguments={
            "interface": "sim",
            "persist": LaunchConfiguration("persist"),
            "rviz": LaunchConfiguration("rviz"),
        }.items()),
    ])
