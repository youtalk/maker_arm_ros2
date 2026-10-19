"""
The Maker Arm in MuJoCo, from the description's URDF with maker-arm-lab's parameters.

The values are ported from the lab, not imported: mj/bench.py is its arm-only plant and
mj/scene.py its grasping scene. A comment names each value's source.
"""

import json
import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
import mujoco
import numpy as np
import xacro

from .motors import MotorBank

# Motor i + 1 drives JOINTS[i]. The right jaw follows the left one through the mimic equality.
JOINTS = (
    'link_002_joint',
    'link_003_joint',
    'link_004_joint',
    'link_005_joint',
    'link_006_joint',
    'link_007_joint',
    'gripper_left_joint',
)
READY = (0.0, -1.79, 2.05, -1.3, -0.22, 0.0)  # lab robots/frames.py HOME_JOINT_POS

PHYSICS_DT = 0.001  # lab mj/bench.py PHYSICS_DT
# Per motor, in the motor frame. Motor 7's values reach the jaw through the gripper map.
# INITIAL: lab robots/frames.py ARM_ARMATURE (kg m^2, UNVERIFIED there), a stability floor;
# the lab's MA2 identification on the real arm measures the rotor inertia.
ARM_ARMATURE = 0.02
VISCOUS_NMS = 0.02  # INITIAL: lab mj/bench.py VISCOUS_NMS, the lab's MA2 fits the real friction
COULOMB_NM = {'RS00': 0.1, 'RS02': 0.2}  # INITIAL: lab mj/bench.py COULOMB_NM, as above
# Lab mj/scene.py CONTACT_SOLREF: 5 ms contacts, so a pinched part does not sink into the jaws.
CONTACT_SOLREF = (0.005, 1.0)
OBJECT_CONDIM = 4  # lab mj/scene.py BRICK_CONDIM: torsional friction, a held part does not spin
DESK_RGBA = (0.5, 0.4, 0.3, 1.0)  # lab mj/scene.py, the desk geom
DESK_HALF_THICKNESS = 0.02  # lab core/layout.py DESK_SIZE: a 40 mm desk
NEAR_CLIP_M = 0.005  # wrist camera near clip; the jaws pass within about 10 mm of the lens
OBSTACLE_RGBA = (0.55, 0.55, 0.55, 1.0)  # lab mj/scene.py, the bin walls
JAW_TRAVEL_M = 0.0524125  # the URDF's jaw limit, lab assets/urdf_prep.py gripper_travel()
# Lab robots/frames.py WRIST_CAM_POS_IN_WRIST and WRIST_CAM_ROT_IN_WRIST: 30 mm off the tool
# axis, 5 mm in front of the motor face, pitched 30 deg back toward grasp_center. The image is
# turned half a turn about the optical axis from the lab's (see build_model).
WRIST_CAM_POS = (0.030, 0.0, -0.118)
WRIST_CAM_PITCH = math.radians(30.0)
WRIST_FOV_DEG = 70.0  # lab core/cameras.py WRIST_FOV_DEG
IMAGE_SIZE = 224  # lab core/cameras.py IMAGE_WIDTH and IMAGE_HEIGHT: square, so fovy is the FOV
DOME_LEVEL = 1.0  # lab mj/scene.py DOME_GAIN * the nominal dome_intensity 1000
SUN_LEVEL = 0.3  # lab mj/scene.py SUN_GAIN * the nominal sun_intensity 2000
SUN_DIR = (-0.29, 0.19, -0.94)  # unit vector, about 20 degrees off vertical
SHADOW_MAP_SIZE = 4096  # lab mj/scene.py: the default 1024 is too coarse for the gripper


def _share(package):
    return Path(get_package_share_directory(package))


def description():
    """Return the plant's URDF text (no desk, no ros2_control) and the mesh directory."""
    share = _share('maker_arm_description')
    doc = xacro.process_file(
        str(share / 'urdf' / 'maker_arm.urdf.xacro'),
        mappings={'desk': 'false', 'use_ros2_control': 'false'},
    )
    return doc.toxml(), share / 'meshes'


def sim_profile():
    """Return the profile that the daemon loads on vcan0. The plant needs its identity map."""
    path = _share('maker_arm_description') / 'config' / 'daemon' / 'sim_profile.json'
    profile = json.loads(path.read_text())
    for j in profile['joints']:
        if (j['direction'], j['offset']) != (1.0, 0.0):
            raise ValueError(f'{path}: joint {j["name"]} is not the identity motor map')
    return profile


def grip_map(profile):
    """
    Return (a, b) of maker-arm-rs GripMap::provisional, jaw gap = a * angle + b.

    The gap is 0 at the gripper's q_lo and 2 * JAW_TRAVEL_M at its q_hi. One jaw moves half the
    gap, so the jaw joint has joint_m_per_rad = a / 2.
    """
    g = profile['joints'][6]
    a = 2 * JAW_TRAVEL_M / (g['q_hi'] - g['q_lo'])
    return a, -a * g['q_lo']


def ready(profile):
    """Return the motor angles of the ready pose, with the jaws half open."""
    g = profile['joints'][6]
    return [*READY, (g['q_lo'] + g['q_hi']) / 2]


def _quat(columns):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.array(columns, float).T.flatten())
    return q


def build_model(urdf_text: str, mesh_dir: Path, scene: dict) -> mujoco.MjModel:
    """
    Build the arm, the desk, the scene's objects and its obstacle as one model.

    The mount, the jaw slide joints and the importer's mimic equality come from the URDF. The
    motor models and the gripper map come from sim_profile().
    """
    # Lab mj/scene.py _urdf_with_compiler: the importer reads the mesh options from this block.
    compiler = (
        f'<mujoco><compiler meshdir="{mesh_dir}" balanceinertia="true" '
        'discardvisual="false" strippath="true"/></mujoco>'
    )
    text = re.sub(r'(<robot[^>]*>)', lambda m: m.group(1) + compiler, urdf_text, count=1)
    spec = mujoco.MjSpec.from_string(text)
    spec.compiler.fusestatic = False  # lab mj/scene.py: keep base_link and grasp_center
    spec.option.timestep = PHYSICS_DT
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # lab mj/bench.py
    # No self-collision (lab mj/scene.py): robot geoms 1/2 and scene geoms 3/3 make every
    # robot-scene and scene-scene pair collide, and no robot-robot pair.
    for geom in spec.geoms:
        if geom.contype or geom.conaffinity:
            geom.contype, geom.conaffinity = 1, 2
            geom.solref = list(CONTACT_SOLREF)
    profile = sim_profile()
    rad_per_m = 2 / grip_map(profile)[0]  # motor 7 per metre of jaw travel: 1 / joint_m_per_rad
    for name, j in zip(JOINTS, profile['joints']):
        s = rad_per_m if name == 'gripper_left_joint' else 1.0
        joint = spec.joint(name)
        joint.armature = ARM_ARMATURE * s * s
        joint.damping = [VISCOUS_NMS * s * s, 0.0, 0.0]  # MuJoCo 3.13: linear, then polynomial
        joint.frictionloss = COULOMB_NM[j['model']] * s

    scene_geoms = [
        # A box, as the lab's mj/scene.py desk, not lab mj/bench.py's plane: the sun's shadow
        # map streaked a plane desk with shadow acne. The top face stays at desk_z.
        spec.worldbody.add_geom(
            name='desk',
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[1.0, 1.0, DESK_HALF_THICKNESS],
            pos=[0.0, 0.0, scene['desk_z'] - DESK_HALF_THICKNESS],
            rgba=list(DESK_RGBA),
        )
    ]
    for o in scene['objects']:
        yaw = o['yaw']
        body = spec.worldbody.add_body(
            name=f'object/{o["name"]}',
            pos=list(o['pos']),
            quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
        )
        body.add_freejoint()
        geom = body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX, size=[s / 2 for s in o['size']], rgba=list(o['rgba'])
        )
        geom.mass = o['mass']
        geom.condim = OBJECT_CONDIM
        scene_geoms.append(geom)
    ob = scene['obstacle']
    scene_geoms.append(
        spec.worldbody.add_geom(
            name=f'obstacle/{ob["name"]}',
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[s / 2 for s in ob['size']],
            pos=list(ob['pos']),
            rgba=list(OBSTACLE_RGBA),
        )
    )
    for geom in scene_geoms:
        geom.contype, geom.conaffinity = 3, 3
        geom.solref = list(CONTACT_SOLREF)

    # MuJoCo cameras look along -z with y up: the ROS optical axes (x right, y down, z forward)
    # turned half a turn about x (lab core/cameras.py ROS_TO_MUJOCO_CAM).
    # The lab's image-down axis is the wrist's +x side (the camera's own side), which put the far
    # end of the desk at the bottom of the frame. Here the image is upright: right and down are
    # the lab's turned half a turn about the optical axis, so the lens pose, grasp_center at
    # the image center and the jaws left and right are unchanged. The lab's mount is
    # WRIST_CAM_UNVERIFIED: no real camera has fixed either way yet.
    p = WRIST_CAM_PITCH
    right = (0.0, -1.0, 0.0)
    down = (-math.cos(p), 0.0, math.sin(p))
    forward = (-math.sin(p), 0.0, -math.cos(p))
    spec.body('link_007').add_camera(
        name='wrist_camera',
        pos=list(WRIST_CAM_POS),
        quat=_quat([right, np.negative(down), np.negative(forward)]),
        fovy=WRIST_FOV_DEG,
    )

    # Lighting: the lab's nominal draw (lab mj/scene.py build_spec). A shadowless top-down dome,
    # half diffuse and half ambient at dome_intensity 1000 (DOME_GAIN 1/1000, neutral white),
    # and a sun at sun_intensity 2000 (SUN_GAIN 0.3/2000) that casts the only shadow. Without
    # lights the default headlight left the wrist camera and any viewer nearly black.
    spec.visual.headlight.active = 0
    spec.visual.quality.shadowsize = SHADOW_MAP_SIZE
    dome = spec.worldbody.add_light(name='dome', pos=[0.3, 0.0, 2.0], dir=[0.0, 0.0, -1.0])
    dome.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    dome.castshadow = False
    dome.diffuse = [0.5 * DOME_LEVEL] * 3
    dome.ambient = [0.5 * DOME_LEVEL] * 3
    dome.specular = [0.05, 0.05, 0.05]
    sun = spec.worldbody.add_light(name='sun', pos=[0.3, 0.0, 2.0], dir=list(SUN_DIR))
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    sun.castshadow = True
    sun.diffuse = [SUN_LEVEL] * 3
    sun.specular = [0.1 * SUN_LEVEL] * 3
    model = spec.compile()
    # The near clip is vis.map.znear x stat.extent, and the box desk (planes do not count)
    # raised the extent from 0.66 to 2.83 m. Pin the clip at NEAR_CLIP_M so the jaws nearest
    # the lens are not cut (lab mj/scene.py notes the same failure at K 150).
    model.vis.map.znear = NEAR_CLIP_M / model.stat.extent
    return model


class Plant:
    """
    The model under the seven motors' MIT law, one PHYSICS_DT step at a time.

    Motor 7's angle maps to the left jaw through the provisional gripper map, and its torque
    becomes a force of tau / joint_m_per_rad. Until a motor is first enabled, the arm stays at
    its start pose: the quickstart has the user support the real arm before the first launch.
    """

    def __init__(self, model, profile, q0):
        self.model, self.data = model, mujoco.MjData(model)
        self.bank = MotorBank([j['model'] for j in profile['joints']])
        a, b = grip_map(profile)
        self.scale = np.array([1.0] * 6 + [2 / a])  # motor units per joint unit
        self.shift = np.array([0.0] * 6 + [-b / a])
        self.qadr = [model.joint(n).qposadr[0] for n in JOINTS]
        self.dadr = [model.joint(n).dofadr[0] for n in JOINTS]
        right = model.joint('gripper_right_joint')
        self.held_q = [*self.qadr, right.qposadr[0]]
        self.held_v = [*self.dadr, right.dofadr[0]]
        q = (np.asarray(q0, float) - self.shift) / self.scale
        self.data.qpos[self.held_q] = [*q, q[6]]
        mujoco.mj_forward(model, self.data)
        self.start = self.data.qpos[self.held_q].copy()
        self.held = True
        self._report([0.0] * 7)

    def motor_q(self):
        return self.data.qpos[self.qadr] * self.scale + self.shift

    def motor_dq(self):
        return self.data.qvel[self.dadr] * self.scale

    def step(self, now_s):
        """Apply the bank's torques for one step, then report the new state to the bank."""
        tau = np.array(self.bank.torques(self.motor_q(), self.motor_dq(), now_s))
        self.data.qfrc_applied[self.dadr] = tau * self.scale
        mujoco.mj_step(self.model, self.data)
        if self.held and any(m.enabled for m in self.bank.motors):
            self.held = False
        if self.held:
            self.data.qpos[self.held_q] = self.start
            self.data.qvel[self.held_v] = 0.0
        self._report(tau)

    def _report(self, tau):
        q, dq = self.motor_q().tolist(), self.motor_dq().tolist()
        for i in range(len(JOINTS)):
            self.bank.feedback(i, q[i], dq[i], float(tau[i]))
