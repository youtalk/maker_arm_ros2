import math

from maker_arm_mujoco import robstride as rs

import pytest

HOST = 0xFD


def approx(a, b, tol):
    assert abs(a - b) <= tol, f'{a} !~ {b} (tol {tol})'


def test_constants_and_model_ranges():
    assert (rs.P_MAX, rs.P_MIN) == (12.57, -12.57)
    assert (rs.KP_MAX, rs.KD_MAX) == (500.0, 5.0)
    assert rs.MODELS['RS00'].t_max == 14.0 and rs.MODELS['RS00'].v_max == 33.0
    # RS02: +-17 Nm / +-44 rad/s. Position +-12.57 assumes firmware > 0.2.2.11.
    rs02 = rs.MODELS['RS02']
    assert (rs02.t_min, rs02.t_max, rs02.v_min, rs02.v_max) == (-17.0, 17.0, -44.0, 44.0)
    assert 'RS99' not in rs.MODELS


def test_u16_roundtrip_and_bounds():
    f = rs.float_to_u16
    assert f(rs.P_MIN, rs.P_MIN, rs.P_MAX) == 0
    assert f(rs.P_MAX, rs.P_MIN, rs.P_MAX) == 65535
    assert f(0.0, rs.P_MIN, rs.P_MAX) == 32768
    assert f(99.0, rs.P_MIN, rs.P_MAX) == 65535  # clamps
    assert f(-99.0, rs.P_MIN, rs.P_MAX) == 0
    approx(rs.u16_to_float(32768, rs.P_MIN, rs.P_MAX), 0.0, 1e-3)
    x = 1.234
    back = rs.u16_to_float(f(x, rs.P_MIN, rs.P_MAX), rs.P_MIN, rs.P_MAX)
    approx(back, x, 25.14 / 65535.0)


def test_round_half_to_even_is_load_bearing():
    # 14.0 Nm on RS02's torque range is an exact tie, 59752.5. Half to even gives
    # 59752, half away from zero gives 59753 (wrong).
    rs02 = rs.MODELS['RS02']
    assert rs.float_to_u16(14.0, rs02.t_min, rs02.t_max) == 59752
    # Ties that both modes agree on, kept as a contrast.
    assert rs.float_to_u16(0.0, rs.P_MIN, rs.P_MAX) == 32768  # 32767.5 -> 32768
    assert rs.float_to_u16(2.5, 0.0, rs.KD_MAX) == 32768  # 32767.5 -> 32768


def test_non_finite_input_is_pinned_not_rejected():
    # NaN lands on the range minimum, infinities clamp to the range ends.
    assert rs.float_to_u16(math.nan, rs.P_MIN, rs.P_MAX) == 0
    assert rs.float_to_u16(math.inf, rs.P_MIN, rs.P_MAX) == 65535
    assert rs.float_to_u16(-math.inf, rs.P_MIN, rs.P_MAX) == 0
    assert rs.float_to_u16(math.inf, 0.0, rs.KP_MAX) == 65535


def test_can_id_and_host_frames():
    # enable frame: comm=3, data16=host 0xFD, target=motor 1
    assert rs.make_can_id(rs.COMM_ENABLE, HOST, 1) == 0x0300FD01
    assert rs.make_can_id(rs.COMM_DISABLE, HOST, 2) == 0x0400FD02
    assert rs.make_can_id(rs.COMM_SET_ZERO, HOST, 3) == 0x0600FD03
    assert rs.make_can_id(rs.COMM_READ_PARAM, HOST, 1) == 0x1100FD01
    assert rs.make_can_id(rs.COMM_WRITE_PARAM, HOST, 1) == 0x1200FD01
    assert rs.split_can_id(0x0300FD01) == (rs.COMM_ENABLE, HOST, 1)


def test_mit_tau_rides_in_the_id_and_the_ranges_decode():
    # 14.0 Nm on RS02 is the tie above: 0xE968 == 59752 in id bits 23..8.
    assert rs.make_can_id(rs.COMM_MIT, 0xE968, 1) == 0x01E96801
    # all-zero command + kp=10, kd=1.0, tau_ff=0: golden from the official SDK.
    pos, vel, kp, kd, tau = rs.decode_mit(
        0x01800001, bytes([0x80, 0x00, 0x80, 0x00, 0x05, 0x1F, 0x33, 0x33]), 'RS00'
    )
    approx(pos, 0.0, 1e-3)
    approx(vel, 0.0, 1e-3)
    approx(kp, 10.0, 500 / 65535)
    approx(kd, 1.0, 5 / 65535)
    approx(tau, 0.0, 1e-3)


def test_an_mit_frame_round_trips_within_one_lsb():
    for model in ('RS00', 'RS02'):
        m = rs.MODELS[model]
        p, v, kp, kd, tau = 1.234, -2.5, 60.0, 4.0, -0.75
        data = b''.join(
            x.to_bytes(2, 'big')
            for x in (
                rs.float_to_u16(p, rs.P_MIN, rs.P_MAX),
                rs.float_to_u16(v, m.v_min, m.v_max),
                rs.float_to_u16(kp, 0.0, rs.KP_MAX),
                rs.float_to_u16(kd, 0.0, rs.KD_MAX),
            )
        )
        can_id = rs.make_can_id(rs.COMM_MIT, rs.float_to_u16(tau, m.t_min, m.t_max), 3)
        got = rs.decode_mit(can_id, data, model)
        lsb = (
            (rs.P_MAX - rs.P_MIN) / 65535,
            (m.v_max - m.v_min) / 65535,
            rs.KP_MAX / 65535,
            rs.KD_MAX / 65535,
            (m.t_max - m.t_min) / 65535,
        )
        for g, want, tol in zip(got, (p, v, kp, kd, tau), lsb):
            approx(g, want, tol)


def test_param_frames_are_little_endian():
    assert rs.param_index(bytes.fromhex('1C70000000000000')) == rs.VBUS  # index little-endian
    w = bytes.fromhex('28700000C8000000')  # value little-endian u32
    assert rs.param_index(w) == rs.CAN_TIMEOUT
    assert int.from_bytes(rs.param_raw(w), 'little') == 200
    # U8 write of run_mode = 0 (MIT): value at byte 4 only.
    assert rs.param_index(bytes.fromhex('0570000000000000')) == rs.RUN_MODE
    assert rs.param_raw(bytes.fromhex('05700000AA000000')) == b'\xaa\0\0\0'
    assert rs.CAN_TIMEOUT_PER_MS == 20


def test_param_reply_matches_the_rust_parse_vector():
    # motor 2 reads back VBUS = 24.5: comm=17, ID bits 15..8 = motor id
    can_id, data = rs.encode_param_reply(2, HOST, rs.VBUS, bytes.fromhex('0000C441'))
    assert can_id == (17 << 24) | (2 << 8) | 0xFD
    assert data == bytes.fromhex('1C7000000000C441')


def test_feedback_golden_vectors():
    # comm=2, mode=2 (Motor), fault=0, motor=1, target=host; temp 34.5 C
    can_id, data = rs.encode_feedback(1, HOST, 0.0, 0.0, 0.0, 34.5, 2, 0, 'RS00')
    assert can_id == (2 << 24) | (2 << 22) | (1 << 8) | 0xFD == 0x028001FD
    assert data == bytes.fromhex('8000800080000159')
    # fault bits ride in id bits 21..16
    can_id, _ = rs.encode_feedback(1, HOST, 1.234, -2.5, 0.75, 34.5, 2, 0x21, 'RS00')
    assert (can_id >> 16) & 0x3F == 0x21
    assert (can_id >> 22) & 0x03 == 2
    assert (can_id >> 8) & 0xFF == 1


@pytest.mark.parametrize('model', ['RS00', 'RS02'])
def test_feedback_inverts_the_decode(model):
    m = rs.MODELS[model]
    can_id, data = rs.encode_feedback(3, HOST, 1.234, -2.5, 0.75, 34.5, 2, 0, model)
    assert can_id == (2 << 24) | (2 << 22) | (3 << 8) | 0xFD
    u = [int.from_bytes(data[i:i + 2], 'big') for i in (0, 2, 4, 6)]
    approx(rs.u16_to_float(u[0], rs.P_MIN, rs.P_MAX), 1.234, 25.14 / 65535)
    approx(rs.u16_to_float(u[1], m.v_min, m.v_max), -2.5, (m.v_max - m.v_min) / 65535)
    approx(rs.u16_to_float(u[2], m.t_min, m.t_max), 0.75, (m.t_max - m.t_min) / 65535)
    assert u[3] / 10.0 == 34.5
