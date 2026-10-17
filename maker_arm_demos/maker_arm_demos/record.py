"""The S3 record: the arm trajectory and the gripper goals that pick_place executed."""

from dataclasses import asdict, dataclass
import json


@dataclass
class Record:
    """Times in s from the record's start; arm positions in rad, jaw positions in m."""

    joint_names: list[str]
    points: list[tuple[float, list[float]]]  # (t, one position per joint name)
    grips: list[tuple[float, float]]  # (t, gripper_left_joint goal)

    def start(self):
        """Return the arm positions of the first point."""
        return self.points[0][1]

    def add(self, trajectory, t):
        """Append a trajectory_msgs/JointTrajectory that starts t s after the record's start."""
        cols = [trajectory.joint_names.index(n) for n in self.joint_names]
        for p in trajectory.points:
            d = p.time_from_start
            self.points.append((t + d.sec + d.nanosec * 1e-9, [p.positions[i] for i in cols]))

    def save(self, path):
        with open(path, 'w') as f:
            json.dump(asdict(self), f)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d['joint_names'], [(t, q) for t, q in d['points']],
                   [(t, x) for t, x in d['grips']])
