"""Pick the scene's object, carry it past the obstacle and place it, in MuJoCo (demo S3).

    ros2 run maker_arm_demos pick_place --seed N --out REC.json [--scene PATH]

It reads the plant's scene file and applies the plant's perturbation for the same seed. The
desk is in the robot description already; the object and the obstacle go into MoveIt's planning
scene. The steps: open, approach (ABOVE_M over the grasp, OMPL), descend (Pilz LIN), close and
attach, lift (LIN), carry past the obstacle (OMPL), place (LIN), open, detach, retreat (LIN).
It records each executed arm trajectory and each gripper goal with its time, and saves the
record to --out after the retreat. Then it reads the object's pose from the plant's TF. The
object is placed when its center lies within PLACED_MM of the place target and above the desk.

It prints exactly one token line on every exit path, then exits through os._exit:
PICK_PLACE_OK {"placed": true, "error_mm": 4.2} or PICK_PLACE_FAIL {...}.
"""

import argparse
import json
import math
import os
import time
import traceback

from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import Pose, PoseStamped
from maker_arm_mujoco.scene import load_scene, perturbed
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import numpy as np
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from maker_arm_demos.random_goals import ARM, current_state, move, moveit_params
from maker_arm_demos.record import Record
from maker_arm_demos.servo_probe import grip, start_node, wait_for

FRAME = 'mount_link'
ABOVE_M = 0.08  # approach, lift and retreat height over the grasp
TIP_M = 0.0342  # the fingertips reach this far below grasp_center (the URDF finger meshes)
TIP_CLEARANCE_M = 0.003  # fingertips over the desk at the grasp
# A jaw goal at the object's half width minus this grips it. A `closed` goal presses a small
# object so hard that it shifts (the plant).
SQUEEZE_M = 0.001
OPEN = 0.050636  # the SRDF's `open`
FINGERS = ['gripper_left', 'gripper_right']
PLACED_MM = 30.0
START_MM = 5.0  # the plant's object lies this close to the scene's pose for the same seed
SETTLE_S = 1.0  # after the retreat, before the object's pose is read


def target(x, y, z, yaw):
    """grasp_center at (x, y, z) in FRAME, the tool pointing down, the jaws across yaw + 90 deg."""
    p = PoseStamped()
    p.header.frame_id = FRAME
    p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
    p.pose.orientation.w, p.pose.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
    return p


def box(name, size, pos, yaw=0.0):
    o = CollisionObject(id=name, operation=CollisionObject.ADD)
    o.header.frame_id = FRAME
    o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pos
    pose.orientation.w, pose.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
    o.primitive_poses = [pose]
    return o


def where(tf, name):
    """Return the plant's object `name` position in FRAME."""
    child = f'object/{name}'
    wait_for(lambda: tf.can_transform(FRAME, child, Time()), f'{child} transform')
    v = tf.lookup_transform(FRAME, child, Time()).transform.translation
    return np.array([v.x, v.y, v.z])


class Recorder:
    """
    Stands in for MoveItPy in move(): it executes each arm trajectory and records it.

    It also sends and records the gripper goals. Times count from its creation.
    """

    def __init__(self, robot, node):
        self.robot, self.node = robot, node
        self.record = Record(ARM, [], [])
        self.t0 = time.monotonic()

    def execute(self, trajectory, controllers):
        t = time.monotonic() - self.t0
        ok = self.robot.execute(trajectory, controllers=controllers)
        if ok:
            self.record.add(trajectory.get_robot_trajectory_msg().joint_trajectory, t)
        return ok

    def grip(self, position):
        self.record.grips.append((time.monotonic() - self.t0, position))
        if not grip(self.node, [position]):
            raise RuntimeError(f'the jaw did not reach {position} m')


def run(robot, node, tf, args):
    from moveit.planning import PlanRequestParameters

    scene = perturbed(load_scene(args.scene), args.seed)
    obj, wall = scene['objects'][0], scene['obstacle']
    off = 1000.0 * np.linalg.norm(where(tf, obj['name']) - obj['pos'])
    if off > START_MM:
        raise RuntimeError(f'the plant has {obj["name"]} {off:.1f} mm from the scene pose of '
                           f'seed {args.seed}: does the launch run another seed?')
    with robot.get_planning_scene_monitor().read_write() as ps:
        ps.apply_collision_object(box(obj['name'], obj['size'], obj['pos'], obj['yaw']))
        ps.apply_collision_object(box(wall['name'], wall['size'], wall['pos']))
        ps.current_state.update()

    def attach(operation):
        a = AttachedCollisionObject(link_name='grasp_center', touch_links=FINGERS)
        a.object = CollisionObject(id=obj['name'], operation=operation)
        with robot.get_planning_scene_monitor().read_write() as ps:
            ps.process_attached_collision_object(a)
            ps.current_state.update()

    arm = robot.get_planning_component('arm')
    current_state(arm)
    lin = PlanRequestParameters(robot, 'lin')
    rec = Recorder(robot, node)

    def go(step, params, x, y, z):
        if not move(rec, arm, params, pose_stamped_msg=target(x, y, z, obj['yaw']),
                    pose_link='grasp_center'):
            raise RuntimeError(f'{step}: MoveIt could not plan or execute it')

    x, y, z = obj['pos']
    grasp_z = scene['desk_z'] + TIP_M + TIP_CLEARANCE_M
    held = grasp_z - z  # grasp_center over the object's center while it is held
    px, py, pz = obj['place']
    rec.grip(OPEN)
    go('approach', None, x, y, grasp_z + ABOVE_M)
    go('descend', lin, x, y, grasp_z)
    rec.grip(obj['size'][1] / 2 - SQUEEZE_M)  # the jaws close along the object's y axis
    attach(CollisionObject.ADD)
    go('lift', lin, x, y, grasp_z + ABOVE_M)
    go('carry', None, px, py, pz + held + ABOVE_M)
    go('place', lin, px, py, pz + held)
    rec.grip(OPEN)
    attach(CollisionObject.REMOVE)
    go('retreat', lin, px, py, pz + held + ABOVE_M)
    rec.record.save(args.out)

    time.sleep(SETTLE_S)
    p = where(tf, obj['name'])
    error = 1000.0 * float(np.linalg.norm(p - obj['place']))
    return {'placed': bool(error <= PLACED_MM and p[2] > scene['desk_z']),
            'error_mm': round(error, 1)}


def main():
    token, payload, code = 'PICK_PLACE_FAIL', {'placed': False}, 1
    try:
        ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
        ap.add_argument('--seed', type=int, required=True)
        ap.add_argument('--out', required=True)
        ap.add_argument('--scene',
                        default=os.path.join(share('maker_arm_mujoco'), 'config', 'scene.yaml'))
        args = ap.parse_args()
        from moveit.planning import MoveItPy

        node = start_node('pick_place')
        tf = Buffer()
        TransformListener(tf, node)
        params = moveit_params()
        params['lin'] = {'plan_request_params': {
            **params['plan_request_params'],
            'planning_pipeline': 'pilz_industrial_motion_planner',
            'planner_id': 'LIN',
        }}
        # It lives until os._exit: deleting a MoveItPy segfaults (MoveIt 2.12).
        robot = MoveItPy(node_name='pick_place_moveit', config_dict=params,
                         provide_planning_service=False)
        payload = run(robot, node, tf, args)
        if payload['placed']:
            token, code = 'PICK_PLACE_OK', 0
    except SystemExit as e:  # argparse errors and --help
        payload = {'placed': False, 'error': f'usage (exit {e.code})'}
        code = 2
    except BaseException as e:  # the token contract holds on a crash too
        traceback.print_exc()
        payload = {'placed': False, 'error': repr(e)}
    print(f'{token} {json.dumps(payload)}', flush=True)
    os._exit(code)
