"""Ruling R42 on SimArm: an arm that stays still at `work` returns to `home`, then parks."""

import importlib.machinery
import importlib.util
import os
import subprocess
import sys
import unittest
from pathlib import Path

import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(__file__))
from v1 import DAEMON_LOG, print_tail, wait_for_controllers  # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
IDLE_S = 3.0
WAIT_S = 60.0


def load(name):
    loader = importlib.machinery.SourceFileLoader(name, str(SCRIPTS / name))
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader(name, loader))
    loader.exec_module(module)
    return module


@pytest.mark.launch_test
def generate_test_description():
    stack = os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py")
    # idle_home_s stays 0 until the arm is at work.
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stack), launch_arguments={
            "interface": "sim", "auto_park": "true"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def done(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=WAIT_S)
    assert future.done(), "no reply"
    return future.result()


class TestIdleHome(unittest.TestCase):

    def test_an_idle_arm_at_work_returns_home_and_parks(self):
        pm, go_home = load("park_monitor"), load("go_home")
        srdf = (Path(share("maker_arm_moveit_config")) / "config" / "maker_arm.srdf").read_text()
        rclpy.init()
        node = rclpy.create_node("idle_home_test")
        try:
            wait_for_controllers()
            move = ActionClient(node, MoveGroup, pm.MOVE)
            self.assertTrue(move.wait_for_server(timeout_sec=WAIT_S))
            # home first: the visit allows the idle return (R42a), the move away arms the park.
            for name in ("home", "work"):
                handle = done(node, move.send_goal_async(
                    pm.move_goal(go_home.state(srdf, "arm", name))))
                res = done(node, handle.get_result_async())
                self.assertEqual(res.result.error_code.val, MoveItErrorCodes.SUCCESS, name)
            subprocess.run(["ros2", "param", "set", "/park_monitor", "idle_home_s", str(IDLE_S)],
                           check=True, timeout=30)
            self.proc_output.assertWaitFor(
                f"idle for {IDLE_S:g} s away from home: returning home", timeout=WAIT_S)
            self.proc_output.assertWaitFor("returned home", timeout=WAIT_S)
            self.proc_output.assertWaitFor("parked: torque off at home", timeout=WAIT_S)
        except BaseException:
            print_tail(DAEMON_LOG)
            raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
