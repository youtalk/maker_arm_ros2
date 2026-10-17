import numpy as np

from maker_arm_demos.servo_probe import judge

HZ = 50
RELEASE = 104.0  # s: the first zero twist, 4 s after the first sample


def track(heading_deg=0.0, coast=0.0):
    """A tip at 0.05 m/s for 4 s, `heading_deg` off +x in the desk plane, then still for 1 s.

    It coasts on for `coast` s past the first zero twist.
    """
    a = np.radians(heading_deg)
    v = 0.05 * np.array([np.cos(a), np.sin(a), 0.0])
    start = np.array([0.17, 0.23, 0.25])
    return [(100.0 + i / HZ, start + v * min(i / HZ, 4.0 + coast)) for i in range(5 * HZ + 1)]


def test_a_straight_track_passes():
    verdict = judge(track(), RELEASE)
    assert verdict['ok']
    assert verdict['angle_deg'] < 0.01
    assert abs(verdict['distance_ratio'] - 1.0) < 0.001
    assert verdict['stop_ms'] < 1.0


def test_a_track_20_degrees_off_fails():
    verdict = judge(track(heading_deg=20.0), RELEASE)
    assert not verdict['ok']
    assert abs(verdict['angle_deg'] - 20.0) < 0.01
    assert abs(verdict['distance_ratio'] - 1.0) < 0.001


def test_a_track_that_coasts_300_ms_fails():
    verdict = judge(track(coast=0.3), RELEASE)
    assert not verdict['ok']
    assert abs(verdict['stop_ms'] - 300.0) < 1.0
    assert verdict['angle_deg'] < 0.01


def test_encoder_noise_after_the_stop_is_not_motion():
    """The tip flips by one encoder count (0.13 mm) between samples once it stands still."""
    noisy = [(t, p + (0.00013 if t > RELEASE and round(t * HZ) % 2 else 0.0))
             for t, p in track()]
    verdict = judge(noisy, RELEASE)
    assert verdict['ok']
    assert verdict['stop_ms'] < 1.0
