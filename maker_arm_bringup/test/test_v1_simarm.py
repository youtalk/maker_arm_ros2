"""V1 on SimArm: MoveIt plans and executes 20 seeded random goals and the gripper, unclamped."""

import os
import sys
import unittest

import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(__file__))
from v1 import DAEMON_LOG, goals, print_tail, status, wait_for_controllers  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    simarm = os.path.join(get_package_share_directory("maker_arm_bringup"), "launch",
                          "simarm.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(simarm)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestV1SimArm(unittest.TestCase):

    # launch_testing prints the launch output. The daemon writes to its own log.
    def test_twenty_goals_and_the_gripper_with_no_clamped_tick(self):
        try:
            wait_for_controllers()
            token, payload = goals(20)
            s = status()
            print(f"STATUS {s}", flush=True)
            self.assertEqual(token, "GOALS_OK", payload)
            self.assertEqual((payload["goals"], payload["ok"], payload["gripper"]),
                             (20, 20, True))
            self.assertEqual(s["clamped"], 0, s)
        except BaseException:
            print_tail(DAEMON_LOG)
            raise
