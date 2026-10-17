import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from maker_arm_demos.record import Record
from maker_arm_demos.replay import check_limits, check_start, confirm, scale

ARM = [f'link_00{i}_joint' for i in range(2, 8)]
READY = [0.0, -1.79, 2.05, -1.3, -0.22, 0.0]
with open(os.path.join(get_package_share_directory('maker_arm_moveit_config'), 'config',
                       'joint_limits.yaml')) as f:
    JOINTS = yaml.safe_load(f)['joint_limits']


def record(points=None, grips=None):
    """READY, then 0.02 rad further on every joint 1 s later, with an open and a grasp."""
    return Record(ARM, points or [(0.0, READY), (1.0, [q + 0.02 for q in READY])],
                  grips or [(0.0, 0.050636), (0.5, 0.014)])


def test_replay_scales_every_time_by_one_over_the_speed():
    slow = scale(record(), 0.5)
    assert [t for t, _ in slow.points] == [0.0, 2.0]
    assert [t for t, _ in slow.grips] == [0.0, 1.0]
    assert [q for _, q in slow.points] == [q for _, q in record().points]
    assert [x for _, x in slow.grips] == [x for _, x in record().grips]


def test_replay_refuses_a_start_farther_than_the_tolerance():
    measured = dict(zip(ARM, READY))
    check_start(measured, record())  # at the start
    check_start({**measured, 'link_004_joint': 2.05 + 0.01}, record())
    with pytest.raises(ValueError, match='link_004_joint'):
        check_start({**measured, 'link_004_joint': 2.05 + 0.03}, record())


def test_replay_refuses_a_point_outside_the_moveit_limits():
    check_limits(record(), JOINTS)
    outside = [0.0, 0.0, 2.05, -1.3, -0.22, 0.0]  # link_003_joint stops at -0.05
    with pytest.raises(ValueError, match='link_003_joint'):
        check_limits(record(points=[(0.0, READY), (5.0, outside)]), JOINTS)
    with pytest.raises(ValueError, match='gripper_left_joint'):
        check_limits(record(grips=[(0.5, 0.06)]), JOINTS)
    with pytest.raises(ValueError, match='link_003_joint'):
        check_limits(record(points=[(0.0, READY), (5.0, [0.0, float('nan'), *READY[2:]])]),
                     JOINTS)


def test_replay_refuses_a_record_faster_than_the_moveit_limits():
    jump = [0.0, -1.79, 2.05, -1.3, -0.22, 1.0]  # link_007_joint 1 rad in 0.1 s: 10 rad/s
    with pytest.raises(ValueError, match='link_007_joint'):
        check_limits(record(points=[(0.0, READY), (0.1, jump)]), JOINTS)
    with pytest.raises(ValueError, match='time'):
        check_limits(record(points=[(0.0, READY), (0.0, READY)]), JOINTS)
    with pytest.raises(ValueError, match='time'):
        check_limits(record(grips=[(2.0, 0.014)]), JOINTS)  # after the arm's last point


def test_anything_but_yes_cancels():
    assert confirm('yes')
    for answer in ('y', 'Y', 'YES', 'Yes', 'yes please', ' yes', '', 'no'):
        assert not confirm(answer), answer
