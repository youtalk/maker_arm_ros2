"""The S3 scene file: the desk, the objects and the obstacle, in mount_link (m, rad, kg)."""

import copy
from pathlib import Path
import random

import yaml

KEYS = ('desk_z', 'objects', 'obstacle')
SHIFT_M = 0.01
TURN_RAD = 0.2


def load_scene(path) -> dict:
    """
    Read a scene file.

    Each object has name, size [3], pos [3], yaw, mass, rgba and place [3].
    """
    scene = yaml.safe_load(Path(path).read_text())
    missing = [k for k in KEYS if k not in scene]
    if missing:
        raise ValueError(f'{path}: missing {missing}')
    return scene


def perturbed(scene: dict, seed: int) -> dict:
    """
    Move each object up to SHIFT_M in x and y and turn it up to TURN_RAD in yaw.

    The plant and the pick-and-place demo both call this, so they agree on the poses.
    """
    rng = random.Random(seed)
    out = copy.deepcopy(scene)
    for o in out['objects']:
        o['pos'][0] += rng.uniform(-SHIFT_M, SHIFT_M)
        o['pos'][1] += rng.uniform(-SHIFT_M, SHIFT_M)
        o['yaw'] += rng.uniform(-TURN_RAD, TURN_RAD)
    return out
