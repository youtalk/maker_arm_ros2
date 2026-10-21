import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from maker_arm_demos.record import Record
from maker_arm_demos.replay import (
    check_clearance, check_limits, check_start, confirm, lowest_tip, scale
)

ARM = [f'link_00{i}_joint' for i in range(2, 8)]
READY = [0.0, -1.79, 2.05, -1.3, 0.0, 0.0]
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


def test_replay_refuses_a_start_just_past_the_tolerance():
    measured = dict(zip(ARM, READY))
    check_start({n: q + 0.019 for n, q in measured.items()}, record())  # every joint within
    with pytest.raises(ValueError, match='link_005_joint'):
        check_start({**measured, 'link_005_joint': -1.3 + 0.021}, record())


def test_replay_refuses_an_unknown_joint_anywhere():
    """ros2_control state interfaces read NaN until the hardware writes them."""
    measured = dict(zip(ARM, READY))
    for name in ('link_004_joint', 'link_007_joint'):  # a middle joint and the last one
        with pytest.raises(ValueError, match=name):
            check_start({**measured, name: float('nan')}, record())
    # The NaN joint is named, not a finite joint that is also too far.
    with pytest.raises(ValueError, match='link_006_joint is nan'):
        check_start({**measured, 'link_003_joint': -1.79 + 0.5,
                     'link_006_joint': float('nan')}, record())


def test_replay_refuses_a_point_outside_the_moveit_limits():
    check_limits(record(), JOINTS)
    outside = [0.0, 0.0, 2.05, -1.3, 0.0, 0.0]  # link_003_joint stops at -0.05
    with pytest.raises(ValueError, match='link_003_joint'):
        check_limits(record(points=[(0.0, READY), (5.0, outside)]), JOINTS)
    with pytest.raises(ValueError, match='gripper_left_joint'):
        check_limits(record(grips=[(0.5, 0.06)]), JOINTS)
    with pytest.raises(ValueError, match='link_003_joint'):
        check_limits(record(points=[(0.0, READY), (5.0, [0.0, float('nan'), *READY[2:]])]),
                     JOINTS)


def test_replay_refuses_a_record_faster_than_the_moveit_limits():
    jump = [0.0, -1.79, 2.05, -1.3, 0.0, 1.0]  # link_007_joint 1 rad in 0.1 s: 10 rad/s
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


def test_replay_refuses_fingertips_closer_to_the_desk_than_the_clearance():
    check_clearance(0.02, 1.0, None)  # at the bar
    with pytest.raises(ValueError, match=r'0\.003 m over the modeled desk at t = 12\.3 s'):
        check_clearance(0.003, 12.3, None)
    with pytest.raises(ValueError, match='--raised M'):  # says how to replay it
        check_clearance(0.003, 12.3, None)
    with pytest.raises(ValueError):
        check_clearance(float('nan'), 0.0, None)


def test_a_raised_arm_adds_its_height_and_stands_at_least_5_cm():
    check_clearance(0.003, 12.3, 0.05)  # 0.053 m over the real desk
    check_clearance(-0.03, 0.0, 0.05)  # 0.02 m, at the bar
    with pytest.raises(ValueError, match='0.010 m'):
        check_clearance(-0.04, 0.0, 0.05)
    for raised in (0.049, 0.0, -0.1, float('nan')):
        with pytest.raises(ValueError, match='--raised'):
            check_clearance(0.5, 0.0, raised)


def test_the_lowest_fingertip_comes_from_fk_over_the_whole_record():
    """FK on the planning model, with no live stack."""
    low, t = lowest_tip(record(points=[(0.0, READY)]))
    assert t == 0.0 and low == pytest.approx(0.1287, abs=0.001)  # tips under grasp_center
    down = [0.0, -2.1, 2.05, -1.3, 0.0, 0.0]  # the shoulder lowers the tool
    low, t = lowest_tip(record(points=[(0.0, READY), (1.0, down), (2.0, READY)]))
    assert t == 1.0 and low == pytest.approx(0.0388, abs=0.001)
