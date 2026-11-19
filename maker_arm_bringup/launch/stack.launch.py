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
with open(os.path.join(DAEMON, "gripper_map.txt")) as f:
    GRIPPER_MAP = f.read().strip()  # the measured map, one line "a,b"
# name -> default, in the order of the xacro args of maker_arm_description, then the others
ARGS = {
    "interface": "sim",
    "executable": "maker-arm-cli",
    "rt": "false",
    "persist": "false",
    "profile": os.path.join(DAEMON, "sim_profile.json"),
    "dynamics": os.path.join(DAEMON, "dynamics_urdf.json"),
    "params": "",
    "residual": "",
    "gripper_map": GRIPPER_MAP,
    "socket": "",
    "log": "",
    "desk": "true",
    "rviz": "false",
    "auto_park": "false",
    "idle_home_s": "0.0",
}
NOT_XACRO = ("rviz", "auto_park", "idle_home_s")
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
        if k not in NOT_XACRO:
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
    # move_group publishes the SRDF on /robot_description_semantic (transient local) for hosts and
    # apps that read it from the topic. robot_state_publisher already publishes /robot_description.
    moveit_params = [description, *moveit.values(),
                     {"use_sim_time": False, "publish_robot_description_semantic": True}]
    # Servo alone lets the fingers touch the desk, so an operator can grasp an object on it.
    # Servo's collision halt ignores the direction, so a finger on the desk froze every command,
    # even one that lifts. The other links still stop at the desk. move_group and park_monitor
    # keep the shared SRDF, so no planned path drags the fingertips through the desk.
    servo_srdf = _text(mc, "maker_arm.srdf").replace("</robot>", "".join(
        f'  <disable_collisions link1="desk_link" link2="gripper_{side}" reason="Never"/>\n'
        for side in ("left", "right")) + "</robot>")
    # The planning limits, so Servo holds the profile-derived limits and not the URDF's.
    servo_params = [
        description,
        {"robot_description_semantic": servo_srdf},
        moveit["robot_description_kinematics"],
        moveit["robot_description_planning"],
        {"moveit_servo": _yaml(mc, "servo.yaml")},
    ]
    controllers_yaml = os.path.join(share("maker_arm_bringup"), "config", "ros2_controllers.yaml")
    rviz_cfg = os.path.join(share("maker_arm_bringup"), "rviz", "maker_arm.rviz")

    nodes = [
        # 100 Hz /tf, not the default 20 Hz: the Servo probe reads the tip's stop from /tf.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[description, {"publish_frequency": 100.0}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[description, controllers_yaml], output="screen"),
        *[Node(package="controller_manager", executable="spawner",
               arguments=[c, "--controller-manager", "/controller_manager"]) for c in CONTROLLERS],
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=moveit_params),
        Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
             parameters=servo_params),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg],
             respawn=True, respawn_delay=2.0,
             parameters=[description, moveit["robot_description_semantic"],
                         moveit["robot_description_kinematics"]],
             condition=IfCondition(cfg["rviz"])),
        # Releases the torque when the arm rests at home (spec 6.9, D9), and returns an idle arm
        # home (ruling R42). Respawned: without it, nothing guards the motors' heat.
        Node(package="maker_arm_bringup", executable="park_monitor", output="screen",
             respawn=True, respawn_delay=2.0,
             parameters=[description, moveit["robot_description_semantic"],
                         {"idle_home_s": ParameterValue(cfg["idle_home_s"], value_type=float)}],
             condition=IfCondition(cfg["auto_park"])),
        # The daemon's status and the motor temperatures on /diagnostics, for rqt_runtime_monitor.
        Node(package="maker_arm_bringup", executable="arm_diagnostics", output="screen",
             respawn=True, respawn_delay=2.0, parameters=[description]),
    ]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + nodes)
