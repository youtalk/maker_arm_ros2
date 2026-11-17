from maker_arm_mujoco import robstride as rs
from maker_arm_mujoco.motors import MotorBank

MODELS = ['RS00', 'RS02', 'RS02', 'RS00', 'RS00', 'RS00', 'RS00']
ENABLE = bytes.fromhex('FFFFFFFFFFFFFFFC')
DISABLE = bytes.fromhex('FFFFFFFFFFFFFFFD')
SET_ZERO = bytes.fromhex('FFFFFFFFFFFFFFFE')
CLEAR = bytes.fromhex('FFFFFFFFFFFFFFFB')
QUERY = bytes.fromhex('FFFFFFFFFFFF00FB')


def read(index):
    return index.to_bytes(2, 'little') + bytes(6)


def write(index, raw):
    return index.to_bytes(2, 'little') + b'\0\0' + raw


def control(pos, kp, kd, tau, model='RS00'):
    """Pack a control payload by the vendor layout."""
    m = rs.MODELS[model]
    q = rs.float_to_uint(pos, rs.P_MIN, rs.P_MAX, 16)
    v = rs.float_to_uint(0.0, m.v_min, m.v_max, 12)
    k = rs.float_to_uint(kp, 0.0, rs.KP_MAX, 12)
    d = rs.float_to_uint(kd, 0.0, rs.KD_MAX, 12)
    t = rs.float_to_uint(tau, m.t_min, m.t_max, 12)
    return bytes([q >> 8, q & 0xFF, v >> 4, (v & 0xF) << 4 | k >> 8, k & 0xFF,
                  d >> 4, (d & 0xF) << 4 | t >> 8, t & 0xFF])


def at_the_sim_park_pose(**kw):
    # SimArm parks each motor at the middle of its joint range: motors 1 and 2 here.
    bank = MotorBank(MODELS, **kw)
    bank.feedback(0, 2.075, 0.0, 0.0, 35.0)
    bank.feedback(1, -0.5225, 0.0, 0.0, 35.0)
    return bank


def one(replies):
    assert len(replies) == 1
    can_id, data = replies[0]
    return f'{can_id:03X} {data.hex().upper()}'


def test_commands_get_feedback_on_the_host_id():
    bank = at_the_sim_park_pose()
    # The default bank is legacy: byte 6 holds only the temperature's top bits, 35.0 C = 0x15E.
    assert one(bank.handle(1, DISABLE, 0.0)) == '0FD 019521800800015E'
    assert one(bank.handle(1, ENABLE, 0.0)) == '0FD 019521800800015E'
    assert one(bank.handle(2, CLEAR, 0.0)) == '0FD 027AAD800800015E'
    assert one(bank.handle(2, SET_ZERO, 0.0)) == '0FD 028000800800015E'


def test_the_params_bank_reports_the_mode():
    bank = at_the_sim_park_pose(params_supported=True)
    assert one(bank.handle(1, DISABLE, 0.0)) == '0FD 019521800800015E'
    assert one(bank.handle(1, ENABLE, 0.0)) == '0FD 019521800800815E'


def test_fault_query_answers_without_clearing():
    bank = MotorBank(MODELS, params_supported=True)
    bank.motors[2].fault_word = 1 << 14
    for _ in range(2):
        assert bank.handle(3, QUERY, 0.0) == [(0xFD, bytes.fromhex('0300400000'))]
    flagged = bank.handle(3, DISABLE, 0.0)[0][1]
    assert flagged[6] & 0x20  # the fault flag shows in the feedback
    assert bank.motors[2].fault_word == 1 << 14
    bank.handle(3, CLEAR, 0.0)  # the state read clears it
    assert bank.handle(3, QUERY, 0.0) == [(0xFD, bytes.fromhex('0300000000'))]


def test_legacy_bank_ignores_parameter_frames():
    bank = MotorBank(MODELS)
    assert bank.handle(0x307, read(rs.RUN_MODE), 0.0) == []
    assert bank.handle(0x407, write(rs.CAN_TIMEOUT, (4000).to_bytes(4, 'little')), 0.0) == []
    bank.handle(7, ENABLE, 0.0)
    bank.handle(7, control(1.0, 60.0, 4.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 100.0)[6] > 1.0  # no watchdog was set


def test_parameter_bank_answers_run_mode_zero():
    bank = MotorBank(MODELS, params_supported=True)
    got = bank.handle(0x303, read(rs.RUN_MODE), 0.0)
    assert got == [(0x303, bytes.fromhex('0570000000000000'))]
    assert bank.handle(0x303, read(rs.VBUS), 0.0) == [(0x303, bytes.fromhex('1C7000000000C041'))]
    assert bank.handle(0x303, read(0x7016), 0.0) == []  # an index it does not hold
    to = (200 * 20).to_bytes(4, 'little')
    assert bank.handle(0x403, write(rs.CAN_TIMEOUT, to), 0.0) == [
        (0x403, bytes.fromhex('28700000A00F0000'))]
    assert bank.handle(0x303, read(rs.CAN_TIMEOUT), 0.0)[0][1][4:] == to


def test_an_unknown_motor_id_or_frame_gets_no_reply():
    bank = MotorBank(MODELS, params_supported=True)
    assert bank.handle(9, DISABLE, 0.0) == []
    assert bank.handle(0, DISABLE, 0.0) == []
    assert bank.handle(0x105, DISABLE, 0.0) == []  # not a command, control or parameter id
    assert bank.handle(1, bytes.fromhex('FFFFFFFFFFFFFF00'), 0.0) == []  # an unknown op
    assert bank.handle(1, DISABLE[:5], 0.0) == []  # a short frame
    assert bank.handle(0x1FFFFFF1, DISABLE, 0.0) == []  # an extended id


def test_control_while_disabled_answers_but_does_not_drive():
    bank = at_the_sim_park_pose()
    got = bank.handle(1, control(1.0, 60.0, 4.0, 0.5), 0.0)
    assert one(got) == '0FD 019521800800015E'  # a feedback reply, the state unchanged
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0) == [0.0] * 7
    bank.handle(1, ENABLE, 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0) == [0.0] * 7  # the frame before was dropped
    bank.handle(1, control(1.0, 60.0, 4.0, 0.5), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] > 1.0
    bank.handle(1, DISABLE, 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] == 0.0


def test_the_mit_law_saturates_at_t_max():
    bank = MotorBank(MODELS)
    bank.handle(1, ENABLE, 0.0)
    bank.handle(1, control(1.0, 60.0, 4.0, 0.5), 0.0)
    got = bank.torques([0.9] + [0.0] * 6, [0.1] + [0.0] * 6, 0.0)[0]
    # kp (p - q) + kd (v - dq) + tau = 60 * 0.1 + 4 * (0 - 0.1) + 0.5 = 6.1, within the wire's LSB
    assert abs(got - 6.1) < 0.05
    bank.handle(1, control(12.0, 500.0, 0.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] == 14.0  # RS00 t_max


def test_the_watchdog_disables_a_silent_motor():
    bank = MotorBank(MODELS, params_supported=True)
    to = (200 * 20).to_bytes(4, 'little')  # CAN_TIMEOUT 200 ms
    bank.handle(0x401, write(rs.CAN_TIMEOUT, to), 0.0)
    bank.handle(1, ENABLE, 0.0)
    bank.handle(1, control(1.0, 60.0, 4.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.1)[0] > 1.0  # 0.1 s of silence is fine
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.25)[0] == 0.0  # 0.25 s is not
    # The motor reports Reset mode, and it stays off until the host enables it again.
    assert bank.handle(1, QUERY, 0.3) == [(0xFD, bytes.fromhex('0100000000'))]
    assert bank.handle(1, DISABLE, 0.3)[0][1][6] >> 6 == 0
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.3)[0] == 0.0


def test_a_zero_can_timeout_means_no_watchdog():
    bank = MotorBank(MODELS, params_supported=True)
    bank.handle(1, ENABLE, 0.0)
    bank.handle(1, control(1.0, 60.0, 4.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 100.0)[0] > 1.0
