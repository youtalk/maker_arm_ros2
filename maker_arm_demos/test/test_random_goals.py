import os
from itertools import islice, repeat
from types import SimpleNamespace

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from maker_arm_demos import random_goals
from maker_arm_demos.random_goals import candidates, current_state, grip, load_limits

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


class FakeHand:
    """A gripper PlanningComponent whose measured jaw reads `jaws` in turn."""

    def __init__(self, jaws):
        self.jaws = iter(jaws)

    def set_start_state_to_current_state(self):
        pass

    def set_goal_state(self, **goal):
        pass

    def plan(self):
        return SimpleNamespace(trajectory=None)

    def get_named_target_state_values(self, name):
        return {'gripper_left_joint': 0.05}

    def get_start_state(self):
        return SimpleNamespace(joint_positions={'gripper_left_joint': next(self.jaws)})


ROBOT = SimpleNamespace(execute=lambda trajectory, controllers: True)


def test_a_gripper_goal_counts_when_the_jaw_arrives():
    assert grip(ROBOT, FakeHand([0.0, 0.03, 0.049]), 'open')


def test_a_gripper_goal_fails_when_the_jaw_never_arrives(monkeypatch):
    monkeypatch.setattr(random_goals, 'JAW_WAIT', 0.2)
    assert not grip(ROBOT, FakeHand(repeat(0.03)), 'open')


class LateArm:
    """An arm PlanningComponent with no current state for its first `late` reads."""

    def __init__(self, late):
        self.reads = iter([None] * late + ['state'])

    def get_start_state(self):
        return next(self.reads, None)


def test_the_current_state_waits_for_moveit():
    assert current_state(LateArm(late=3)) == 'state'


def test_no_current_state_fails_with_a_reason(monkeypatch):
    monkeypatch.setattr(random_goals, 'STATE_WAIT', 0.2)
    with pytest.raises(TimeoutError, match='no current robot state'):
        current_state(LateArm(late=100))


class Plans:
    """An arm PlanningComponent whose plan() returns `results` in turn (True = success)."""

    def __init__(self, *results):
        self.results = iter(results)
        self.calls = 0

    def set_start_state_to_current_state(self):
        pass

    def set_goal_state(self, **goal):
        pass

    def plan(self):
        self.calls += 1
        ok = next(self.results)

        class Plan:
            trajectory = None
            error_code = SimpleNamespace(val=-1)

            def __bool__(self):
                return ok

        return Plan()


def test_a_plan_that_fails_twice_then_succeeds_counts_as_ok():
    arm = Plans(False, False, True)
    assert random_goals.try_move(ROBOT, arm) is None
    assert arm.calls == 3


def test_a_plan_that_fails_every_try_records_a_plan_failure():
    arm = Plans(False, False, False)
    assert random_goals.try_move(ROBOT, arm) == ('plan', -1)
    assert arm.calls == random_goals.PLAN_TRIES


def test_an_execution_failure_is_recorded_and_not_retried():
    class Failed:
        status = 'ABORTED'

        def __bool__(self):
            return False

    arm = Plans(True, True)
    robot = SimpleNamespace(execute=lambda trajectory, controllers: Failed())
    assert random_goals.try_move(robot, arm) == ('execute', 'ABORTED')
    assert arm.calls == 1
