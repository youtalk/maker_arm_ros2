import math
from pathlib import Path

from maker_arm_mujoco import robstride as rs
from maker_arm_mujoco.model import (
    build_model, description, JOINTS, Plant, READY, ready, sim_profile
)
from maker_arm_mujoco.scene import load_scene, perturbed
import mujoco
import numpy as np

SCENE = Path(__file__).resolve().parents[1] / 'config' / 'scene.yaml'
HOST = 0xFD
MOUNT_RPY = (math.pi / 2, 0.0, math.pi)  # the lab's assets/urdf_prep.py MOUNT_RPY


def model_of_the_scene():
    return build_model(*description(), load_scene(SCENE))


def mit_frame(motor, pos, kp, kd, model):
    m = rs.MODELS[model]
    data = b''.join(
        x.to_bytes(2, 'big')
        for x in (
            rs.float_to_u16(pos, rs.P_MIN, rs.P_MAX),
            rs.float_to_u16(0.0, m.v_min, m.v_max),
            rs.float_to_u16(kp, 0.0, rs.KP_MAX),
            rs.float_to_u16(kd, 0.0, rs.KD_MAX),
        )
    )
    return rs.make_can_id(rs.COMM_MIT, rs.float_to_u16(0.0, m.t_min, m.t_max), motor), data


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


def test_the_mit_hold_keeps_the_ready_pose_under_gravity():
    profile = sim_profile()
    target = ready(profile)  # the jaws half open
    plant = Plant(model_of_the_scene(), profile, target)
    for i, j in enumerate(profile['joints']):
        plant.bank.handle(rs.make_can_id(rs.COMM_ENABLE, HOST, i + 1), bytes(8), 0.0)
        plant.bank.handle(*mit_frame(i + 1, target[i], j['kp'], j['kd'], j['model']), 0.0)
    for k in range(2000):
        plant.step(k * 0.001)
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
