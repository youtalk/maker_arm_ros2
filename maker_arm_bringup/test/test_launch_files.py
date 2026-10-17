import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

LAUNCH = Path(__file__).resolve().parents[1] / "launch"


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f"{name}.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("name", ["stack", "simarm", "real", "sim"])
def test_each_launch_file_builds_a_description(name):
    assert load(name).generate_launch_description().entities


def test_the_servo_node_keeps_the_conduit_names():
    nodes = [e for e in load("stack").generate_launch_description().entities if isinstance(e, Node)]
    servo = [n for n in nodes if n.node_package == "moveit_servo"]
    assert len(servo) == 1 and servo[0]._Node__node_name == "servo_node"
    assert {n.node_executable for n in nodes} >= {"move_group", "ros2_control_node", "spawner"}


def test_servo_reads_the_planning_limits():
    nodes = [e for e in load("stack").generate_launch_description().entities if isinstance(e, Node)]
    servo = next(n for n in nodes if n.node_package == "moveit_servo")
    names = {"".join(t.text for t in key) for p in servo._Node__parameters for key in p}
    for j in range(2, 8):
        assert f"robot_description_planning.joint_limits.link_00{j}_joint.max_velocity" in names


def test_real_launch_names_the_session_for_a_missing_file():
    real = load("real")
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {"interface": "can0", "residual": "", "rt": "false", "persist": "false", "rviz": "false",
         **{k: "/nonexistent/" + v[0] for k, v in real.FILES.items()}})
    with pytest.raises(RuntimeError) as e:
        real.launch_setup(ctx)
    msg = str(e.value)
    for session in ("MS5 session 2", "MS5 session 4", "MS5 session 5"):
        assert session in msg
    for f in ("real_profile_caps050.json", "gripper_map.txt", "dynamics_ident.json", "params_shipped.json"):
        assert f in msg


def test_real_launch_passes_residual_and_rt_to_the_stack(tmp_path):
    real = load("real")
    declared = [e.name for e in real.generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)]
    assert {"residual", "rt"} <= set(declared)
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {"interface": "can0", "residual": "/r.json", "rt": "true", "persist": "false",
         "rviz": "false"})
    for k, v in real.FILES.items():
        (tmp_path / v[0]).write_text("-0.03,0.01")
        ctx.launch_configurations[k] = str(tmp_path / v[0])
    (stack,) = real.launch_setup(ctx)
    args = dict(stack.launch_arguments)
    assert args["residual"] == "/r.json" and args["rt"] == "true"


def test_sim_starts_the_plant_and_waits_for_plant_ready():
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessIO

    entities = load("sim").generate_launch_description().entities
    plants = [e for e in entities if isinstance(e, Node) and e.node_package == "maker_arm_mujoco"]
    assert len(plants) == 1 and plants[0].node_executable == "plant"
    handlers = [e.event_handler for e in entities if isinstance(e, RegisterEventHandler)]
    assert any(isinstance(h, OnProcessIO) for h in handlers)
