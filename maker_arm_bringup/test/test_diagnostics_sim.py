"""/diagnostics on SimArm: the 8 `maker_arm: ` statuses, with the motor temperatures."""

import os
import sys
import time
import unittest

import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(__file__))
from v1 import DAEMON_LOG, print_tail, wait_for_controllers  # noqa: E402

WAIT_S = 60.0


@pytest.mark.launch_test
def generate_test_description():
    stack = os.path.join(share("maker_arm_bringup"), "launch", "stack.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(stack), launch_arguments={
            "interface": "sim", "auto_park": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDiagnostics(unittest.TestCase):

    def test_diagnostics_carry_the_arm_statuses_with_temperatures(self):
        rclpy.init()
        node = rclpy.create_node("diagnostics_test")
        got = []
        node.create_subscription(DiagnosticArray, "/diagnostics", got.append, 10)
        try:
            wait_for_controllers()
            deadline = time.monotonic() + WAIT_S
            arm = []
            while not arm and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
                # controller_manager publishes its own statuses: keep the arm's.
                arm = [s for m in got for s in m.status if s.name.startswith("maker_arm: ")]
                got.clear()
            self.assertEqual(len(arm), 8, [s.name for s in arm])
            by_name = {s.name: s for s in arm}
            self.assertIn("maker_arm: daemon", by_name)
            self.assertEqual(by_name["maker_arm: daemon"].hardware_id, "sim")
            joint = by_name["maker_arm: link_002_joint"]
            values = {kv.key: kv.value for kv in joint.values}
            self.assertEqual(values["motor id"], "1")
            self.assertGreater(float(values["temperature C"]), 0.0)
            self.assertEqual(joint.message[-2:], " C")
        except BaseException:
            print_tail(DAEMON_LOG)
            raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
