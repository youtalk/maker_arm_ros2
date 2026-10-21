"""Servo the tip along +x and move the gripper, as Conduit will (check V2).

    ros2 run maker_arm_demos servo_probe

It stands in for Conduit until Conduit's arm view exists. The arm first moves to START
through MoveIt. Then the probe switches Servo to TWIST and publishes TwistStamped in
mount_link at 50 Hz: +x at 0.05 m/s for 4 s, then zero twists for 1 s. All along, it reads
mount_link to grasp_center from TF. Then it sends GripperCommand goals: open, then closed.

The pass rule is Conduit 3.0 spec G2. The tip moves along +x within 15 degrees and within
30 % of 0.2 m. It stops (speed under 5 mm/s) within 200 ms of the first zero twist. Both
gripper goals succeed, and each counts only when the jaw gets within the controller's
goal tolerance.

It prints exactly one token line on every exit path, then exits through os._exit:
SERVO_OK {"angle_deg": 1.2, "distance_ratio": 0.97, "stop_ms": 60.0, "gripper": true}
or SERVO_FAIL {...}.
"""

import argparse
import copy
import json
import os
import threading
import time
import traceback

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from maker_arm_demos.random_goals import current_state, move, moveit_params

HZ = 50
SPEED = 0.05  # m/s along +x of mount_link
PUSH = 4.0  # s of +x twists, then HOLD s of zero twists
HOLD = 1.0
TARGET = SPEED * PUSH  # 0.2 m
MAX_ANGLE = 15.0  # deg
MAX_DISTANCE_ERROR = 0.3  # of TARGET
STOP_SPEED = 0.005  # m/s
# One encoder count moves the tip about 0.13 mm. Two counts over 50 ms read 5.2 mm/s, so a
# 50 ms window tested the noise (V2). Over 100 ms they read 2.6 mm/s, and a real 5 mm/s coast
# still reads 5 mm/s.
WINDOW = 0.1  # s
MAX_STOP_MS = 200.0
# Servo halts when the Jacobian's condition number reaches hard_stop_singularity_threshold
# (30, the default). At `ready` it is 35.2. From START, a search over this arm's model keeps it
# between 14.7 and 20.1 along the whole 0.2 m line, and the tip 0.25 m above the desk.
START = [-1.32, -0.84, 0.58, 0.39, 0.72, 0.04]
WAIT = 10.0  # s for each server, topic, transform and goal


def start_node(name, **init):
    """Start rclpy with init's options and spin a new node in a daemon thread. Return the node."""
    rclpy.init(**init)
    node = rclpy.create_node(name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return node


def judge(track, t_release):
    """Score a tip track [(t in s, xyz in mount_link)] by Conduit 3.0 spec G2.

    The tip stops at the start of the first WINDOW after the release from which every later
    WINDOW, up to the end of the track, is slower than STOP_SPEED. A window's speed is its
    chord over its time: a path length would read one encoder count of noise as motion.
    """
    t = np.array([s[0] for s in track])
    p = np.array([s[1] for s in track])
    d = p[-1] - p[0]
    dist = float(np.linalg.norm(d))
    angle = float(np.degrees(np.arccos(np.clip(d[0] / dist, -1.0, 1.0)))) if dist else 180.0
    stop_ms = None
    for i in np.flatnonzero(t >= t_release)[::-1]:  # from the end back to the last fast window
        j = np.searchsorted(t, t[i] + WINDOW)
        if j == len(t):
            continue  # less than one WINDOW of track is left
        if np.linalg.norm(p[j] - p[i]) / (t[j] - t[i]) >= STOP_SPEED:
            break
        stop_ms = round(1000.0 * (t[i] - t_release), 1)
    ratio = dist / TARGET
    return {
        'angle_deg': round(angle, 2),
        'distance_ratio': round(ratio, 3),
        'stop_ms': stop_ms,
        'ok': (angle <= MAX_ANGLE and abs(ratio - 1.0) <= MAX_DISTANCE_ERROR
               and stop_ms is not None and stop_ms <= MAX_STOP_MS),
    }


def wait_for(ready, what):
    deadline = time.monotonic() + WAIT
    while not ready():
        if time.monotonic() > deadline:
            raise TimeoutError(f'no {what} in {WAIT} s')
        time.sleep(0.02)


def result(future, what):
    wait_for(future.done, what)
    return future.result()


def servo(node, tf):
    """Push the tip +x through Servo and return judge() of the tip track."""
    switch = node.create_client(ServoCommandType, '/servo_node/switch_command_type')
    wait_for(switch.service_is_ready, 'switch_command_type service')
    request = ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST)
    if not result(switch.call_async(request), 'switch_command_type reply').success:
        raise RuntimeError('Servo refused the switch to TWIST')
    pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
    wait_for(lambda: pub.get_subscription_count() > 0, 'subscriber on delta_twist_cmds')
    wait_for(lambda: tf.can_transform('mount_link', 'grasp_center', Time()), 'tip transform')

    msg = TwistStamped()
    msg.header.frame_id = 'mount_link'
    track, push = [], round(PUSH * HZ)
    tick = time.monotonic()
    for n in range(push + round(HOLD * HZ)):
        now = node.get_clock().now()
        if n == push:
            t_release = now.nanoseconds * 1e-9
        msg.header.stamp = now.to_msg()
        msg.twist.linear.x = SPEED if n < push else 0.0
        pub.publish(msg)
        sample = tf.lookup_transform('mount_link', 'grasp_center', Time())
        t = Time.from_msg(sample.header.stamp).nanoseconds * 1e-9
        if not track or t > track[-1][0]:  # the same joint state can come back twice
            v = sample.transform.translation
            track.append((t, np.array([v.x, v.y, v.z])))
        tick += 1.0 / HZ
        time.sleep(max(0.0, tick - time.monotonic()))
    return judge(track, t_release)


def grip(node, positions):
    """Send one GripperCommand goal per jaw position. True when the jaw reaches every one."""
    client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
    wait_for(client.server_is_ready, 'gripper_cmd action server')
    reached = []
    for position in positions:
        goal = GripperCommand.Goal()
        goal.command.position = position
        handle = result(client.send_goal_async(goal), 'gripper goal reply')
        if not handle.accepted:
            reached.append(False)
            continue
        res = result(handle.get_result_async(), 'gripper result')
        reached.append(res.status == GoalStatus.STATUS_SUCCEEDED and res.result.reached_goal)
    return all(reached)


def main():
    token, payload, code = 'SERVO_FAIL', {}, 1
    try:
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.parse_args()
        from moveit.planning import MoveItPy

        node = start_node('servo_probe')
        tf = Buffer()
        TransformListener(tf, node)
        # It lives until os._exit: deleting a MoveItPy segfaults (MoveIt 2.12).
        robot = MoveItPy(node_name='servo_probe_moveit', config_dict=moveit_params(),
                         provide_planning_service=False)
        arm = robot.get_planning_component('arm')
        start = copy.copy(current_state(arm))
        start.set_joint_group_positions('arm', np.array(START))
        start.update()
        if not move(robot, arm, robot_state=start):
            raise RuntimeError('the move to START failed')
        hand = robot.get_planning_component('gripper')
        jaw = [hand.get_named_target_state_values(name)['gripper_left_joint']
               for name in ('open', 'closed')]

        payload = servo(node, tf)
        ok = payload.pop('ok')
        payload['gripper'] = grip(node, jaw)
        if ok and payload['gripper']:
            token, code = 'SERVO_OK', 0
    except SystemExit as e:  # argparse errors and --help
        payload = {'error': f'usage (exit {e.code})'}
        code = 2
    except BaseException as e:  # the token contract holds on a crash too
        traceback.print_exc()
        payload = {**payload, 'error': repr(e)}  # keep the track's verdict if it has one
    print(f'{token} {json.dumps(payload)}', flush=True)
    os._exit(code)
