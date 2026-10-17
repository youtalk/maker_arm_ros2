"""Plan and execute seeded random joint goals and the gripper through MoveIt (check V1).

    ros2 run maker_arm_demos random_goals --count 20 --seed 0 [--sag]

The goals are drawn uniformly inside joint_limits.yaml, and each one the planning scene
reports valid is kept. Then the gripper moves to `open` and `closed`. Each counts only when
the measured jaw gets within 2 mm of it in 3 s. With --sag, the arm moves to `ready`, waits
2 s and reports the distance in mm between the commanded and the measured grasp_center.

It prints exactly one token line on every exit path, then exits through os._exit:
GOALS_OK {"goals": 20, "ok": 20, "gripper": true, "sag_mm": null} or GOALS_FAIL {...}.
A failed goal adds a "failures" list: its index, the stage (plan or execute) and the code.
"""

import argparse
import copy
import json
import os
import time
import traceback
from itertools import islice

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory as share

ARM = [f'link_00{i}_joint' for i in range(2, 8)]
# ponytail: rejection sampling stops after this many draws per goal, then the run fails.
DRAWS_PER_GOAL = 100
JAW_TOL = 0.002  # m, the gripper controller's goal_tolerance
JAW_WAIT = 3.0  # s
STATE_WAIT = 10.0  # s for MoveIt's first complete joint state
# RRTConnect can time out on a slow runner for a goal it solves on a retry.
PLAN_TRIES = 3


def load_limits(path):
    with open(path) as f:
        joints = yaml.safe_load(f)['joint_limits']
    return [(joints[n]['min_position'], joints[n]['max_position']) for n in ARM]


def candidates(limits, seed):
    """Yield arm joint states drawn uniformly inside limits, without end."""
    lo, hi = np.array(limits).T
    rng = np.random.default_rng(seed)
    while True:
        yield rng.uniform(lo, hi)


def _path(name):
    return os.path.join(share('maker_arm_moveit_config'), 'config', name)


def _config(name):
    with open(_path(name)) as f:
        return f.read() if name.endswith('.srdf') else yaml.safe_load(f)


def moveit_params():
    """Build the MoveIt parameters as stack.launch.py does, for one OMPL pipeline."""
    import xacro

    planning = _config('joint_limits.yaml')
    urdf = os.path.join(share('maker_arm_description'), 'urdf', 'maker_arm.urdf.xacro')
    return {
        'robot_description': xacro.process_file(urdf).toxml(),
        'robot_description_semantic': _config('maker_arm.srdf'),
        'robot_description_kinematics': _config('kinematics.yaml'),
        'robot_description_planning': planning,
        'planning_pipelines': {'pipeline_names': ['ompl']},
        'ompl': _config('ompl_planning.yaml'),
        **_config('moveit_controllers.yaml'),
        'plan_request_params': {
            'planning_pipeline': 'ompl',
            'planning_attempts': 1,
            'planning_time': 5.0,
            'max_velocity_scaling_factor': planning['default_velocity_scaling_factor'],
            'max_acceleration_scaling_factor': planning['default_acceleration_scaling_factor'],
        },
    }


def current_state(component):
    """Return the measured state. Right after start, MoveIt can have none yet, so wait."""
    deadline = time.monotonic() + STATE_WAIT
    while (state := component.get_start_state()) is None:
        if time.monotonic() > deadline:
            raise TimeoutError(f'no current robot state from MoveIt in {STATE_WAIT} s')
        time.sleep(0.1)
    return state


def try_move(robot, component, **goal):
    """Plan, then execute. Return None on success, else (stage, MoveIt code or status).

    A failed plan is retried with a fresh plan() up to PLAN_TRIES. A failed execution is not:
    the arm moved, and a retry would hide a real problem.
    """
    component.set_start_state_to_current_state()
    component.set_goal_state(**goal)
    for _ in range(PLAN_TRIES):
        plan = component.plan()
        if plan:
            break
    else:
        code = getattr(plan, 'error_code', None)
        return 'plan', getattr(code, 'val', code)
    result = robot.execute(plan.trajectory, controllers=[])
    if result:
        return None
    return 'execute', str(getattr(result, 'status', result))


def move(robot, component, **goal):
    return try_move(robot, component, **goal) is None


def grip(robot, hand, name):
    """Move the gripper to a named state. True only when the measured jaw arrives.

    The controller can report success on a stall before the jaw gets there.
    """
    if not move(robot, hand, configuration_name=name):
        return False
    goal = hand.get_named_target_state_values(name)['gripper_left_joint']
    deadline = time.monotonic() + JAW_WAIT
    while time.monotonic() < deadline:
        if abs(hand.get_start_state().joint_positions['gripper_left_joint'] - goal) < JAW_TOL:
            return True
        time.sleep(0.05)
    return False


def run(robot, args):
    limits = load_limits(_path('joint_limits.yaml'))
    arm = robot.get_planning_component('arm')
    # The measured state: the gripper stays where the planner will see it.
    measured = current_state(arm)

    goals = []
    with robot.get_planning_scene_monitor().read_only() as scene:
        for q in islice(candidates(limits, args.seed), DRAWS_PER_GOAL * args.count):
            state = copy.copy(measured)
            state.set_joint_group_positions('arm', q)
            state.update()
            if scene.is_state_valid(state, 'arm'):
                goals.append(state)
                if len(goals) == args.count:
                    break

    failures = []
    for i, g in enumerate(goals):
        if failure := try_move(robot, arm, robot_state=g):
            failures.append({'goal': i, 'stage': failure[0], 'code': failure[1]})
    ok = len(goals) - len(failures)
    hand = robot.get_planning_component('gripper')
    opened = grip(robot, hand, 'open')
    closed = grip(robot, hand, 'closed')
    gripper = opened and closed

    sag = None
    if args.sag:
        if not move(robot, arm, configuration_name='ready'):
            raise RuntimeError('the move to ready failed')
        time.sleep(2.0)
        measured = arm.get_start_state()
        commanded = copy.copy(measured)
        commanded.set_to_default_values('arm', 'ready')
        commanded.update()
        tip = [s.get_global_link_transform('grasp_center')[:3, 3] for s in (commanded, measured)]
        sag = round(1000.0 * float(np.linalg.norm(tip[0] - tip[1])), 3)
    return {'goals': args.count, 'ok': ok, 'gripper': gripper, 'sag_mm': sag,
            **({'failures': failures} if failures else {})}


def main():
    token, payload, code = 'GOALS_FAIL', {}, 1
    try:
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.add_argument('--count', type=int, default=20)
        ap.add_argument('--seed', type=int, default=0)
        ap.add_argument('--sag', action='store_true')
        args = ap.parse_args()
        from moveit.planning import MoveItPy

        # It lives until os._exit: deleting a MoveItPy segfaults (MoveIt 2.12).
        robot = MoveItPy(node_name='random_goals', config_dict=moveit_params(),
                         provide_planning_service=False)
        payload = run(robot, args)
        if payload['ok'] == args.count and payload['gripper']:
            token, code = 'GOALS_OK', 0
    except SystemExit as e:  # argparse errors and --help
        payload = {'error': f'usage (exit {e.code})'}
        code = 2
    except BaseException as e:  # the token contract holds on a crash too
        traceback.print_exc()
        payload = {'error': repr(e)}
    print(f'{token} {json.dumps(payload)}', flush=True)
    os._exit(code)
