"""Replay an S3 record on the arm at half speed, from a checked start pose (demo S3).

    ros2 run maker_arm_demos replay REC.json [--approach]

It refuses a record with a point outside joint_limits.yaml: a position past a limit, or a joint
faster than its max_velocity. It refuses to start unless every arm joint lies within
START_TOL_RAD of the record's start. With --approach, it plans a MoveIt move to the start at
scaling 0.1 instead, and moves only after the operator types yes. Then it prints the replay's
duration and joint range and asks for yes again. It sends the arm points as one
FollowJointTrajectory goal, with every time divided by REPLAY_SPEED, and each gripper goal at
its scaled time. On Ctrl-C, SIGTERM, SIGHUP or any error while a goal runs, it cancels the arm
goal, and the controller holds the arm where it is.

It prints exactly one token line on every exit path, then exits through os._exit:
REPLAY_OK {"duration_s": 120.4, "points": 812, "grips": 3} or REPLAY_FAIL {"reason": "..."}.
"""

import argparse
import copy
from dataclasses import replace
import json
import os
import signal
import time
import traceback

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from maker_arm_demos.random_goals import _config, ARM, current_state, moveit_params
from maker_arm_demos.record import Record
from maker_arm_demos.servo_probe import result, start_node, wait_for

START_TOL_RAD = 0.02  # INITIAL: spec section 6.6
REPLAY_SPEED = 0.5  # INITIAL: spec section 6.6
APPROACH_SCALING = 0.1  # MoveIt velocity and acceleration scaling of the move to the start
LEAD_S = 1.0  # a goal reaches its first point this long after it starts, from the measured pose
RESULT_WAIT_S = 10.0  # past the trajectory's end, for the controller's result
CANCEL_WAIT_S = 3.0
JAW = 'gripper_left_joint'
KEEP = []  # MoveItPy objects stay here until os._exit: deleting one segfaults (MoveIt 2.12)


def scale(record, speed):
    """Return the record with every time divided by speed."""
    return replace(record, points=[(t / speed, q) for t, q in record.points],
                   grips=[(t / speed, x) for t, x in record.grips])


def check_limits(record, joints):
    """
    Raise ValueError unless the record lies inside joints, joint_limits.yaml's joint_limits.

    Every arm position and gripper goal lies inside its position limits, no arm joint moves
    faster than its max_velocity, the point times rise from 0, and every gripper goal falls
    inside the arm's time span (a later one would never be sent).
    """
    if record.joint_names != ARM:
        raise ValueError(f'the record moves {record.joint_names}, not {ARM}')
    if not record.points:
        raise ValueError('the record has no points')
    values = [(t, n, x) for t, q in record.points for n, x in zip(ARM, q, strict=True)]
    for t, n, x in values + [(t, JAW, x) for t, x in record.grips]:
        lo, hi = joints[n]['min_position'], joints[n]['max_position']
        if not lo <= x <= hi:  # also false for NaN
            raise ValueError(f'{n} = {x} at t = {t} s lies outside [{lo}, {hi}]')
    times = [t for t, _ in record.points]
    if not 0 <= times[0] or any(not a < b for a, b in zip(times, times[1:])):
        raise ValueError('the point times do not rise from 0')
    for (a, qa), (b, qb) in zip(record.points, record.points[1:]):
        for n, xa, xb in zip(ARM, qa, qb):
            v, vmax = abs(xb - xa) / (b - a), joints[n]['max_velocity']
            if v > vmax:
                raise ValueError(f'{n} moves at {v:.2f} rad/s at t = {a} s, over {vmax} rad/s')
    if any(not 0 <= t <= times[-1] for t, _ in record.grips):
        raise ValueError(f'a gripper goal time lies outside the arm points, 0 to {times[-1]} s')


def check_start(measured, record):
    """Raise ValueError unless every arm joint of measured ({name: rad}) is near the start."""
    off = {n: abs(measured[n] - q) for n, q in zip(record.joint_names, record.start())}
    worst = max(off, key=off.get)
    if not off[worst] <= START_TOL_RAD:  # also false for NaN
        raise ValueError(f'{worst} is {off[worst]:.3f} rad from the record start, over '
                         f'{START_TOL_RAD} rad (--approach moves it there)')


def confirm(answer):
    """Only the exact word yes confirms."""
    return answer == 'yes'


def ask(prompt):
    print(prompt, flush=True)  # on its own line: the token line must start a line
    if not confirm(input()):
        raise ValueError('cancelled: the operator did not type yes')


def measured(node):
    """Return the next measured joint state as {name: position}."""
    msgs = []
    sub = node.create_subscription(JointState, '/joint_states', msgs.append, 10)
    try:
        wait_for(lambda: msgs, 'message on /joint_states')
    finally:
        node.destroy_subscription(sub)
    return dict(zip(msgs[0].name, msgs[0].position))


def stop(future):
    """Cancel the goal that future sends. Return True when no goal of it runs any more."""
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):  # a second one must not skip it
        signal.signal(s, signal.SIG_IGN)
    deadline = time.monotonic() + CANCEL_WAIT_S
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if not future.done() or not future.result().accepted:
        return future.done()  # refused, or no reply yet: then it may still run
    cancel = future.result().cancel_goal_async()
    while not cancel.done() and time.monotonic() < deadline + CANCEL_WAIT_S:
        time.sleep(0.01)
    ended = (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED)
    return cancel.done() and cancel.result().return_code in ended


def follow(node, record):
    """
    Send the record's points as one FollowJointTrajectory goal and its gripper goals on time.

    The goal starts now and reaches the first point LEAD_S later. On any exception, Ctrl-C
    included, it cancels the goal, and the controller holds the arm where it is.
    """
    arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    hand = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
    wait_for(arm.server_is_ready, 'follow_joint_trajectory action server')
    if record.grips:
        wait_for(hand.server_is_ready, 'gripper_cmd action server')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = record.joint_names
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=q, time_from_start=Duration(seconds=LEAD_S + t).to_msg())
        for t, q in record.points]
    future = arm.send_goal_async(goal)
    sent = []
    try:
        handle = result(future, 'arm goal reply')
        if not handle.accepted:
            raise RuntimeError('arm_controller refused the trajectory')
        start = time.monotonic()
        done = handle.get_result_async()
        for t, x in record.grips:
            while not done.done() and time.monotonic() < start + LEAD_S + t:
                time.sleep(0.005)
            if done.done():
                break
            grip = GripperCommand.Goal()
            grip.command.position = x
            sent.append(hand.send_goal_async(grip))
        deadline = start + LEAD_S + record.points[-1][0] + RESULT_WAIT_S
        while not done.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f'no arm_controller result {RESULT_WAIT_S} s after the end')
            if not arm.server_is_ready():
                raise RuntimeError('the arm_controller action server is gone')
            time.sleep(0.005)
    except BaseException as e:
        state = 'no arm goal runs' if stop(future) else 'the arm goal MAY STILL RUN'
        raise RuntimeError(f'{e!r}; {state}') from e
    res = done.result()
    if res.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f'arm_controller ended the goal with status {res.status}: '
                           f'{res.result.error_string}')
    refused = sum(not (f.done() and f.result().accepted) for f in sent)
    if refused:
        raise RuntimeError(f'{refused} of {len(sent)} gripper goals were not accepted')


def approach(node, record):
    """Move to the record's start along a MoveIt plan at APPROACH_SCALING, after a typed yes."""
    from moveit.planning import MoveItPy, PlanRequestParameters

    params = moveit_params()
    params['approach'] = {'plan_request_params': {
        **params['plan_request_params'],
        'max_velocity_scaling_factor': APPROACH_SCALING,
        'max_acceleration_scaling_factor': APPROACH_SCALING,
    }}
    robot = MoveItPy(node_name='replay_moveit', config_dict=params,
                     provide_planning_service=False)
    KEEP.append(robot)
    arm = robot.get_planning_component('arm')
    goal = copy.copy(current_state(arm))
    goal.set_joint_group_positions('arm', np.array(record.start()))
    goal.update()
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal)
    plan = arm.plan(single_plan_parameters=PlanRequestParameters(robot, 'approach'))
    if not plan:
        raise RuntimeError('MoveIt found no move to the record start')
    path = Record(ARM, [], [])
    path.add(plan.trajectory.get_robot_trajectory_msg().joint_trajectory, 0.0)
    print(f'The arm is away from the record start. MoveIt moves it there in '
          f'{LEAD_S + path.points[-1][0]:.1f} s at scaling {APPROACH_SCALING}.')
    ask('Type yes to move the arm to the record start:')
    follow(node, path)


def run(args):
    record = Record.load(args.record)
    check_limits(record, _config('joint_limits.yaml')['joint_limits'])
    # No rclpy signal handlers: on Ctrl-C they end the context, and the cancel cannot go out.
    node = start_node('replay', signal_handler_options=SignalHandlerOptions.NO)
    try:
        check_start(measured(node), record)
    except ValueError:
        if not args.approach:
            raise
        approach(node, record)
        check_start(measured(node), record)
    slow = scale(record, REPLAY_SPEED)
    duration = LEAD_S + slow.points[-1][0]
    print(f'Replay of {args.record} at speed {REPLAY_SPEED}: {duration:.1f} s, '
          f'{len(slow.points)} points, {len(slow.grips)} gripper goals.')
    for name, column in zip(ARM, zip(*(q for _, q in slow.points))):
        print(f'  {name}: {min(column):+.3f} to {max(column):+.3f} rad')
    if slow.grips:
        jaw = [x for _, x in slow.grips]
        print(f'  {JAW}: {min(jaw):.4f} to {max(jaw):.4f} m')
    ask('Type yes to replay it on the arm:')
    check_start(measured(node), record)  # the arm can have moved while the prompt waited
    follow(node, slow)
    return {'duration_s': round(duration, 1), 'points': len(slow.points),
            'grips': len(slow.grips)}


def main():
    token, payload, code = 'REPLAY_FAIL', {}, 1
    try:
        # All three raise KeyboardInterrupt, which cancels a running goal. SIGINT too: a shell
        # starts a background job with SIGINT ignored.
        for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(s, signal.default_int_handler)
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.add_argument('record')
        ap.add_argument('--approach', action='store_true')
        args = ap.parse_args()
        payload = run(args)
        token, code = 'REPLAY_OK', 0
    except SystemExit as e:  # argparse errors and --help
        payload = {'reason': f'usage (exit {e.code})'}
        code = 2
    except BaseException as e:  # the token contract holds on a crash too
        if not isinstance(e, ValueError):
            traceback.print_exc()
        payload = {'reason': repr(e)}
    print(f'{token} {json.dumps(payload)}', flush=True)
    os._exit(code)
