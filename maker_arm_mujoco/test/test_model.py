import math
from pathlib import Path

from maker_arm_mujoco import robstride as rs
from maker_arm_mujoco.model import (
    build_model, description, grip_map, JOINTS, Plant, READY, ready, sim_profile
)
from maker_arm_mujoco.scene import load_scene, perturbed
import mujoco
import numpy as np
import pytest

SCENE = Path(__file__).resolve().parents[1] / 'config' / 'scene.yaml'
ENABLE = bytes.fromhex('FFFFFFFFFFFFFFFC')
MOUNT_RPY = (math.pi / 2, 0.0, math.pi)  # the lab's assets/urdf_prep.py MOUNT_RPY


def model_of_the_scene():
    return build_model(*description(), load_scene(SCENE))


def mit_frame(motor, pos, kp, kd, model):
    m = rs.MODELS[model]
    q = rs.float_to_uint(pos, rs.P_MIN, rs.P_MAX, 16)
    v = rs.float_to_uint(0.0, m.v_min, m.v_max, 12)
    k = rs.float_to_uint(kp, 0.0, rs.KP_MAX, 12)
    d = rs.float_to_uint(kd, 0.0, rs.KD_MAX, 12)
    t = rs.float_to_uint(0.0, m.t_min, m.t_max, 12)
    data = bytes([q >> 8, q & 0xFF, v >> 4, (v & 0xF) << 4 | k >> 8, k & 0xFF,
                  d >> 4, (d & 0xF) << 4 | t >> 8, t & 0xFF])
    return motor, data


def test_the_model_compiles_with_the_seven_driven_joints_and_the_camera():
    model = model_of_the_scene()
    assert model.opt.timestep == 0.001
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    kinds = [model.joint(n).type[0] for n in JOINTS]
    assert kinds == [mujoco.mjtJoint.mjJNT_HINGE] * 6 + [mujoco.mjtJoint.mjJNT_SLIDE]
    # The right jaw follows the left through the importer's mimic equality.
    (eq,) = range(model.neq)
    pair = {model.joint(model.eq_obj1id[eq]).name, model.joint(model.eq_obj2id[eq]).name}
    assert pair == {'gripper_left_joint', 'gripper_right_joint'}
    cam = model.camera('wrist_camera')
    assert model.body(cam.bodyid[0]).name == 'link_007'
    # Upright image: the camera's up axis (MuJoCo +y) is the wrist's +x side tilted toward -z,
    # the lab's image-down axis, and its right axis (MuJoCo +x) is the wrist's -y.
    axes = np.zeros(9)
    mujoco.mju_quat2Mat(axes, cam.quat)
    axes = axes.reshape(3, 3)
    p = math.radians(30.0)
    assert np.allclose(axes[:, 1], (math.cos(p), 0.0, -math.sin(p)), atol=1e-9), axes[:, 1]
    assert np.allclose(axes[:, 0], (0.0, -1.0, 0.0), atol=1e-9), axes[:, 0]
    # base_link is mounted Z-up by the lab's MOUNT_RPY.
    want = np.zeros(4)
    mujoco.mju_euler2Quat(want, np.array(MOUNT_RPY), 'XYZ')
    assert abs(np.dot(model.body('base_link').quat, want)) > 1 - 1e-9
    assert model.body('object/block').jntnum[0] == 1  # a free body
    assert model.geom('obstacle/wall').bodyid[0] == 0  # static
    # No self-collision: with the jaws closed the fingers overlap and still make no contact.
    data = mujoco.MjData(model)
    data.qpos[[model.joint(n).qposadr[0] for n in JOINTS[:6]]] = READY
    mujoco.mj_forward(model, data)
    # The world frame is mount_link: grasp_center is where the description test puts it.
    grasp = data.xpos[model.body('grasp_center').id]
    assert np.allclose(grasp, (0.2763, 0.0, 0.1607), atol=0.002), grasp
    robot = {b for b in range(1, model.nbody) if not model.body(b).name.startswith('object/')}
    for i in range(data.ncon):
        c = data.contact[i]
        assert not {model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]} <= robot


def hold(plant, profile, target, steps):
    """Enable the seven motors, send each one MIT frame at target, then step the plant."""
    for i, j in enumerate(profile['joints']):
        plant.bank.handle(i + 1, ENABLE, 0.0)
        plant.bank.handle(*mit_frame(i + 1, target[i], j['kp'], j['kd'], j['model']), 0.0)
    for k in range(steps):
        plant.step(k * 0.001)


def test_the_mit_hold_keeps_the_ready_pose_under_gravity():
    profile = sim_profile()
    target = ready(profile)  # the jaws half open
    plant = Plant(model_of_the_scene(), profile, target)
    hold(plant, profile, target, 2000)
    assert not plant.held  # the motors held the arm, not the start-pose support
    err = np.abs(plant.motor_q() - target)
    print('worst joint error', err.max(), 'rad, per joint', err.round(4))
    assert err.max() < 0.05


def test_perturbed_is_seeded_and_bounded():
    scene = load_scene(SCENE)
    a, b = perturbed(scene, 3), perturbed(scene, 3)
    assert a == b
    assert perturbed(scene, 4) != a
    assert scene == load_scene(SCENE)  # the input is not changed
    for seed in range(50):
        moved = perturbed(scene, seed)
        assert moved['obstacle'] == scene['obstacle']
        assert moved['desk_z'] == scene['desk_z']
        for o, p in zip(scene['objects'], moved['objects']):
            assert abs(p['pos'][0] - o['pos'][0]) <= 0.01 + 1e-12
            assert abs(p['pos'][1] - o['pos'][1]) <= 0.01 + 1e-12
            assert p['pos'][2] == o['pos'][2]
            assert abs(p['yaw'] - o['yaw']) <= 0.2 + 1e-12
            assert {k: v for k, v in p.items() if k not in ('pos', 'yaw')} == {
                k: v for k, v in o.items() if k not in ('pos', 'yaw')
            }


def test_grip_map_equals_the_daemons_provisional_map():
    a, b = grip_map()

    def jaw(angle):  # maker-arm-rs GripMap::to_joint_m: one jaw is half the gap
        return (a * angle + b) / 2

    # The values of maker-arm-rs's the_provisional_grip_map_is_the_vendor_preview test.
    assert abs(jaw(0.006713)) < 1e-12
    assert abs(jaw(-2.078984) - 0.0524125) < 1e-12
    assert abs(jaw(math.radians(-96.0)) - 0.042274) < 1e-5


def test_the_plant_reports_j5_negated():
    profile = sim_profile()
    target = ready(profile)
    target[4] = 0.3  # motor 5
    plant = Plant(model_of_the_scene(), profile, target)
    joint5 = plant.model.joint('link_006_joint').qposadr[0]
    assert plant.data.qpos[joint5] == pytest.approx(-0.3)
    hold(plant, profile, target, 500)  # a wrong torque sign drives joint 5 away
    assert abs(plant.data.qpos[joint5] + 0.3) < 0.05
    assert abs(plant.motor_q()[4] - 0.3) < 0.05
    assert plant.bank.motors[4].q == plant.motor_q()[4]  # the feedback the bank sends
