"""Internal: the whole stack (robot_state_publisher, ros2_control, MoveIt, Servo)."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DAEMON = os.path.join(share("maker_arm_description"), "config", "daemon")
# name -> default, in the order of the xacro args of maker_arm_description
ARGS = {
    "interface": "sim",
    "executable": "maker-arm-cli",
    "rt": "false",
    "persist": "false",
    "profile": os.path.join(DAEMON, "sim_profile.json"),
    "dynamics": os.path.join(DAEMON, "dynamics_urdf.json"),
    "params": "",
    "residual": "",
    "gripper_map": "",
    "socket": "",
    "log": "",
    "desk": "true",
    "rviz": "false",
}
CONTROLLERS = ("joint_state_broadcaster", "arm_controller", "gripper_controller")


def _yaml(pkg, name):
    with open(os.path.join(share(pkg), "config", name)) as f:
        return yaml.safe_load(f)


def _text(pkg, name):
    with open(os.path.join(share(pkg), "config", name)) as f:
        return f.read()


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in ARGS}
    xacro_cmd = ["xacro ", os.path.join(share("maker_arm_description"), "urdf", "maker_arm.urdf.xacro")]
    for k in ARGS:
        if k != "rviz":
            xacro_cmd += [f" {k}:=", cfg[k]]
    description = {"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}

    mc = "maker_arm_moveit_config"
    limits = _yaml(mc, "joint_limits.yaml")
    limits.update(_yaml(mc, "pilz_cartesian_limits.yaml"))
    moveit = {
        "robot_description_semantic": {"robot_description_semantic": _text(mc, "maker_arm.srdf")},
        "robot_description_kinematics": {"robot_description_kinematics": _yaml(mc, "kinematics.yaml")},
        "robot_description_planning": {"robot_description_planning": limits},
        "pipelines": {
            "planning_pipelines": {"pipeline_names": ["ompl", "pilz_industrial_motion_planner"]},
            "default_planning_pipeline": "ompl",
            "ompl": _yaml(mc, "ompl_planning.yaml"),
            "pilz_industrial_motion_planner": _yaml(mc, "pilz_industrial_motion_planner_planning.yaml"),
        },
        "controllers": _yaml(mc, "moveit_controllers.yaml"),
    }
    moveit_params = [description, *moveit.values(), {"use_sim_time": False}]
    servo_params = [
        description,
        moveit["robot_description_semantic"],
        moveit["robot_description_kinematics"],
        {"moveit_servo": _yaml(mc, "servo.yaml")},
    ]
    controllers_yaml = os.path.join(share("maker_arm_bringup"), "config", "ros2_controllers.yaml")
    rviz_cfg = os.path.join(share("maker_arm_bringup"), "rviz", "maker_arm.rviz")

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[description, controllers_yaml], output="screen"),
        *[Node(package="controller_manager", executable="spawner",
               arguments=[c, "--controller-manager", "/controller_manager"]) for c in CONTROLLERS],
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=moveit_params),
        Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
             parameters=servo_params),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             parameters=[description, moveit["robot_description_semantic"],
                         moveit["robot_description_kinematics"]],
             condition=IfCondition(cfg["rviz"])),
    ]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + nodes)
