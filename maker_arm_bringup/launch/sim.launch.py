"""The stack on the MuJoCo plant over vcan0: the plant first, the stack on its PLANT_READY line."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DAEMON = os.path.join(share("maker_arm_description"), "config", "daemon")


def generate_launch_description():
    plant = Node(
        package="maker_arm_mujoco", executable="plant", output="screen",
        parameters=[{"interface": "vcan0", "camera_hz": 30.0,
                     "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int)},
                    {"scene": LaunchConfiguration("scene"), "start": LaunchConfiguration("start")}])
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py")),
        launch_arguments={
            "interface": "vcan0",
            "profile": LaunchConfiguration("profile"),
            "dynamics": os.path.join(DAEMON, "dynamics_urdf.json"),
            "persist": LaunchConfiguration("persist"),
            "rviz": LaunchConfiguration("rviz"),
            "auto_park": LaunchConfiguration("auto_park"),
        }.items())

    def on_stdout(event):
        if "PLANT_READY" in event.text.decode(errors="replace"):
            return [stack]

    return LaunchDescription([
        DeclareLaunchArgument("persist", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # On vcan0 a release ends the daemon, so a park there ends the stack's arm.
        DeclareLaunchArgument("auto_park", default_value="true"),
        DeclareLaunchArgument(
            "scene", default_value=os.path.join(share("maker_arm_mujoco"), "config", "scene.yaml")),
        DeclareLaunchArgument("seed", default_value="0"),
        # The plant reports motor 5 negated, as the real arm does: a profile with the vendor
        # calibration. real_profile_caps05.json and start:=rest rehearse the real arm's first launch.
        DeclareLaunchArgument(
            "profile", default_value=os.path.join(DAEMON, "real_profile_nominal.json")),
        DeclareLaunchArgument("start", default_value="ready"),
        plant,
        RegisterEventHandler(OnProcessIO(target_action=plant, on_stdout=on_stdout)),
        # The plant's own message is already on the screen; a dead plant ends the launch.
        RegisterEventHandler(OnProcessExit(target_action=plant, on_exit=[
            LogInfo(msg="plant exited, shutting down"),
            EmitEvent(event=Shutdown(reason="the MuJoCo plant exited"))])),
    ])
