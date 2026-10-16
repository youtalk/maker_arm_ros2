from maker_arm_mujoco import robstride as rs
from maker_arm_mujoco.motors import MotorBank

HOST = 0xFD
MODELS = ['RS00', 'RS02', 'RS02', 'RS00', 'RS00', 'RS00', 'RS00']


def frame(comm, motor, data=bytes(8)):
    return rs.make_can_id(comm, HOST, motor), data


def write(index, raw):
    return index.to_bytes(2, 'little') + b'\0\0' + raw


def mit(motor, pos, kp, kd, tau, model='RS00'):
    m = rs.MODELS[model]
    data = b''.join(
        x.to_bytes(2, 'big')
        for x in (
            rs.float_to_u16(pos, rs.P_MIN, rs.P_MAX),
            rs.float_to_u16(0.0, m.v_min, m.v_max),
            rs.float_to_u16(kp, 0.0, rs.KP_MAX),
            rs.float_to_u16(kd, 0.0, rs.KD_MAX),
        )
    )
    return rs.make_can_id(rs.COMM_MIT, rs.float_to_u16(tau, m.t_min, m.t_max), motor), data


def bank_at_the_sim_park_pose():
    # SimArm parks each motor at the middle of its joint range: motors 1 and 2 here.
    bank = MotorBank(MODELS)
    bank.feedback(0, 2.075, 0.0, 0.0, 35.0)
    bank.feedback(1, -0.5225, 0.0, 0.0, 35.0)
    return bank


def one(replies):
    assert len(replies) == 1
    can_id, data = replies[0]
    return f'{can_id:08X} {data.hex().upper()}'


def test_connect_sequence_gets_the_replies_the_session_expects():
    # Vectors recorded by a throwaway Rust test that drove SimArm (maker_arm_v1) with the
    # frames of Session::connect and Session::enable (maker-arm-rs 8c3e8b0 era sim.rs).
    bank = bank_at_the_sim_park_pose()
    t = 0.0
    want = {
        1: ('020001FD 952180008000015E', '028001FD 952180008000015E', '028001FD 800080008000015E'),
        2: ('020002FD 7AAD80008000015E', '028002FD 7AAD80008000015E', '028002FD 800080008000015E'),
    }
    for m in (1, 2):
        probe, enable, zero = want[m]
        assert one(bank.handle(*frame(rs.COMM_DISABLE, m), t)) == probe
        # RUN_MODE=0 and CAN_TIMEOUT=200 ms (4000 counts) writes are fire and forget.
        assert bank.handle(*frame(rs.COMM_WRITE_PARAM, m, write(rs.RUN_MODE, bytes(4))), t) == []
        to = (200 * 20).to_bytes(4, 'little')
        assert bank.handle(*frame(rs.COMM_WRITE_PARAM, m, write(rs.CAN_TIMEOUT, to)), t) == []
        # Read back.
        read = bank.handle(*frame(rs.COMM_READ_PARAM, m, write(rs.RUN_MODE, bytes(4))), t)
        assert one(read) == f'1100{m:02X}FD 0570000000000000'
        read = bank.handle(*frame(rs.COMM_READ_PARAM, m, write(rs.CAN_TIMEOUT, bytes(4))), t)
        assert one(read) == f'1100{m:02X}FD 28700000A00F0000'
        read = bank.handle(*frame(rs.COMM_READ_PARAM, m, write(rs.VBUS, bytes(4))), t)
        assert one(read) == f'1100{m:02X}FD 1C7000000000C041'
        assert one(bank.handle(*frame(rs.COMM_DISABLE, m), t)) == probe  # second probe
        assert one(bank.handle(*frame(rs.COMM_ENABLE, m), t)) == enable
        assert one(bank.handle(*frame(rs.COMM_SET_ZERO, m, b'\x01' + bytes(7)), t)) == zero


def test_an_unknown_motor_or_comm_type_gets_no_reply():
    bank = MotorBank(MODELS)
    assert bank.handle(*frame(rs.COMM_DISABLE, 9), 0.0) == []
    assert bank.handle(*frame(rs.COMM_DISABLE, 0), 0.0) == []
    assert bank.handle(*frame(22, 2), 0.0) == []  # COMM_SAVE is not implemented
    assert bank.handle(*frame(rs.COMM_READ_PARAM, 2, write(0x7016, bytes(4))), 0.0) == []


def test_the_mit_law_saturates_at_t_max():
    bank = MotorBank(MODELS)
    bank.handle(*frame(rs.COMM_ENABLE, 1), 0.0)
    bank.handle(*mit(1, 1.0, 60.0, 4.0, 0.5), 0.0)
    got = bank.torques([0.9] + [0.0] * 6, [0.1] + [0.0] * 6, 0.0)[0]
    # kp (p - q) + kd (v - dq) + tau = 60 * 0.1 + 4 * (0 - 0.1) + 0.5 = 6.1, within the wire's LSB
    assert abs(got - 6.1) < 0.05
    bank.handle(*mit(1, 12.0, 500.0, 0.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] == 14.0  # RS00 t_max


def test_a_disabled_motor_gives_zero_torque():
    bank = MotorBank(MODELS)
    bank.handle(*mit(1, 1.0, 60.0, 4.0, 0.5), 0.0)  # MIT while disabled replies but is ignored
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0) == [0.0] * 7
    bank.handle(*frame(rs.COMM_ENABLE, 1), 0.0)
    bank.handle(*mit(1, 1.0, 60.0, 4.0, 0.5), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] > 1.0
    bank.handle(*frame(rs.COMM_DISABLE, 1), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.0)[0] == 0.0


def test_the_watchdog_disables_a_silent_motor():
    bank = MotorBank(MODELS)
    to = (200 * 20).to_bytes(4, 'little')  # CAN_TIMEOUT 200 ms
    bank.handle(*frame(rs.COMM_WRITE_PARAM, 1, write(rs.CAN_TIMEOUT, to)), 0.0)
    bank.handle(*frame(rs.COMM_ENABLE, 1), 0.0)
    bank.handle(*mit(1, 1.0, 60.0, 4.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.1)[0] > 1.0  # 0.1 s of silence is fine
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.25)[0] == 0.0  # 0.25 s is not
    # The motor reports Reset mode, and it stays off until the host enables it again.
    mode = (one(bank.handle(*frame(rs.COMM_DISABLE, 1), 0.3)).split()[0])
    assert (int(mode, 16) >> 22) & 3 == 0
    assert bank.torques([0.0] * 7, [0.0] * 7, 0.3)[0] == 0.0


def test_a_zero_can_timeout_means_no_watchdog():
    bank = MotorBank(MODELS)
    bank.handle(*frame(rs.COMM_ENABLE, 1), 0.0)
    bank.handle(*mit(1, 1.0, 60.0, 4.0, 0.0), 0.0)
    assert bank.torques([0.0] * 7, [0.0] * 7, 100.0)[0] > 1.0
