from maker_arm_mujoco import robstride as rs

import pytest


def approx(a, b, tol):
    assert abs(a - b) <= tol, f'{a} !~ {b} (tol {tol})'


def test_constants_and_model_ranges():
    assert rs.HOST_ID == 0xFD
    assert (rs.P_MAX, rs.P_MIN) == (12.57, -12.57)
    assert (rs.KP_MAX, rs.KD_MAX) == (500.0, 5.0)
    assert rs.MODELS['RS00'].t_max == 14.0 and rs.MODELS['RS00'].v_max == 33.0
    rs02 = rs.MODELS['RS02']
    assert (rs02.t_min, rs02.t_max, rs02.v_min, rs02.v_max) == (-17.0, 17.0, -44.0, 44.0)
    assert 'RS99' not in rs.MODELS


def test_scaling_rounds_halves_to_even_and_clamps():
    f = rs.float_to_uint
    assert f(rs.P_MIN, rs.P_MIN, rs.P_MAX, 16) == 0
    assert f(rs.P_MAX, rs.P_MIN, rs.P_MAX, 16) == 65535
    assert f(0.0, rs.P_MIN, rs.P_MAX, 16) == 32768  # 32767.5 goes to the even neighbour
    assert f(0.0, -33.0, 33.0, 12) == 2048  # 2047.5
    assert f(99.0, rs.P_MIN, rs.P_MAX, 16) == 65535
    assert f(-99.0, rs.P_MIN, rs.P_MAX, 16) == 0
    assert f(5.0, 0.0, 6.0, 2) == 2  # 2.5 goes down to the even 2, half away would give 3
    x = 1.234
    back = rs.uint_to_float(f(x, rs.P_MIN, rs.P_MAX, 16), rs.P_MIN, rs.P_MAX, 16)
    approx(back, x, 25.14 / 65535.0)


@pytest.mark.parametrize('model,pos,vel,kp,kd,tau,want', [
    ('RS02', 0.0, 0.0, 500.0, 5.0, 0.0, '8000800ffffff800'),
    ('RS00', 0.0, 0.0, 0.0, 0.0, 0.0, '8000800000000800'),
    ('RS00', 1.0, 0.0, 60.0, 4.0, 0.5, '8a2e8001ebccc849'),
    ('RS02', -1.0, 0.0, 150.0, 4.5, 3.0, '75d18004cce66969'),
    ('RS00', 0.3, -1.2, 30.0, 2.0, -0.4, '830e7b50f66667c5'),
])
def test_control_decodes_the_vendor_vector(model, pos, vel, kp, kd, tau, want):
    m = rs.MODELS[model]
    got = rs.decode_control(bytes.fromhex(want), model)
    lsb = (25.14 / 65535, (m.v_max - m.v_min) / 4095, 500 / 4095, 5 / 4095,
           (m.t_max - m.t_min) / 4095)
    for g, w, tol in zip(got, (pos, vel, kp, kd, tau), lsb):
        approx(g, w, tol)


def test_a_command_payload_is_not_a_control_payload():
    assert rs.decode_control(bytes.fromhex('FFFFFFFFFFFFFFFC'), 'RS00') is None
    assert rs.decode_control(bytes.fromhex('FFFFFFFFFFFF00FB'), 'RS00') is None


def test_p0_feedback_bytes_round_trip():
    # The real arm's motor 1 at rest, captured in P0: its firmware leaves the status nibble 0.
    got = rs.encode_feedback(
        1, 6.144118562600138, -0.0402930402930437, -0.0034188034188034067, 24.0, 0,
        False, False, legacy=True)
    assert got == bytes.fromhex('01BE907FD7FF00F0')


def test_status_bits_and_temperature_ride_in_byte_6_and_7():
    got = rs.encode_feedback(5, 0.0, 0.0, 0.0, 41.3, 2, True, True, legacy=False)
    assert got == bytes.fromhex('058000800800B19D')
    # A legacy motor drops mode and flags, and keeps the temperature.
    got = rs.encode_feedback(5, 0.0, 0.0, 0.0, 41.3, 2, True, True, legacy=True)
    assert got == bytes.fromhex('058000800800019D')


@pytest.mark.parametrize('model', ['RS00', 'RS02'])
def test_feedback_inverts_the_ranges(model):
    m = rs.MODELS[model]
    d = rs.encode_feedback(3, 1.234, -2.5, 0.75, 34.5, 2, False, False, False, model)
    approx(rs.uint_to_float(d[1] << 8 | d[2], rs.P_MIN, rs.P_MAX, 16), 1.234, 25.14 / 65535)
    approx(rs.uint_to_float(d[3] << 4 | d[4] >> 4, m.v_min, m.v_max, 12), -2.5,
           (m.v_max - m.v_min) / 4095)
    approx(rs.uint_to_float((d[4] & 0xF) << 8 | d[5], m.t_min, m.t_max, 12), 0.75,
           (m.t_max - m.t_min) / 4095)


def test_fault_status_is_five_bytes():
    assert rs.encode_fault_status(3, 1 << 14) == bytes.fromhex('0300400000')
    assert rs.encode_fault_status(1, 0) == bytes.fromhex('0100000000')


def test_param_frames_are_little_endian():
    assert rs.param_index(bytes.fromhex('1C70000000000000')) == rs.VBUS
    w = bytes.fromhex('28700000C8000000')
    assert rs.param_index(w) == rs.CAN_TIMEOUT
    assert int.from_bytes(rs.param_raw(w), 'little') == 200
    assert rs.CAN_TIMEOUT_PER_MS == 20


def test_param_replies_ride_the_read_and_write_ids():
    raw = bytes.fromhex('10270000')
    assert rs.encode_param_reply(7, rs.CAN_TIMEOUT, raw, False) == (
        0x307, bytes.fromhex('2870000010270000'))
    assert rs.encode_param_reply(7, rs.CAN_TIMEOUT, raw, True)[0] == 0x407
