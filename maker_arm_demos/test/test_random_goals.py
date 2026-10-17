import os
from itertools import islice

import numpy as np
from ament_index_python.packages import get_package_share_directory

from maker_arm_demos.random_goals import candidates, load_limits

LIMITS = load_limits(os.path.join(
    get_package_share_directory('maker_arm_moveit_config'), 'config', 'joint_limits.yaml'))


def test_every_state_lies_inside_the_limits():
    lo, hi = np.array(LIMITS).T
    assert len(LIMITS) == 6
    for q in islice(candidates(LIMITS, 0), 1000):
        assert q.shape == (6,)
        assert np.all(lo <= q) and np.all(q <= hi)


def test_the_same_seed_gives_the_same_states():
    a = list(islice(candidates(LIMITS, 0), 20))
    b = list(islice(candidates(LIMITS, 0), 20))
    c = list(islice(candidates(LIMITS, 1), 20))
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)
