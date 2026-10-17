"""RobStride private CAN protocol, the motor side. Mirrors maker-arm-protocol (Rust)."""

import math
from typing import NamedTuple

P_MIN, P_MAX = -12.57, 12.57
KP_MAX = 500.0
KD_MAX = 5.0

COMM_MIT = 1
COMM_FEEDBACK = 2
COMM_ENABLE = 3
COMM_DISABLE = 4
COMM_SET_ZERO = 6
COMM_READ_PARAM = 17
COMM_WRITE_PARAM = 18

RUN_MODE = 0x7005  # u8
VBUS = 0x701C  # f32
CAN_TIMEOUT = 0x7028  # u32, 50 us per count
CAN_TIMEOUT_PER_MS = 20


class Model(NamedTuple):
    """The u16 mapping ranges of one motor model."""

    t_max: float
    v_max: float

    @property
    def t_min(self):
        return -self.t_max

    @property
    def v_min(self):
        return -self.v_max


MODELS = {'RS00': Model(14.0, 33.0), 'RS02': Model(17.0, 44.0)}


def float_to_u16(x, lo, hi):
    """Clamp and quantize like the Rust codec: round half to even, NaN to 0."""
    if x != x:
        return 0
    x = min(max(x, lo), hi)
    return round((x - lo) * 65535.0 / (hi - lo))


def u16_to_float(v, lo, hi):
    return v * (hi - lo) / 65535.0 + lo


def make_can_id(comm, data16, target):
    return ((comm & 0x1F) << 24) | (data16 << 8) | target


def split_can_id(can_id):
    """Return (comm, data16, target)."""
    return (can_id >> 24) & 0x1F, (can_id >> 8) & 0xFFFF, can_id & 0xFF


def decode_mit(can_id, data, model):
    """Return (pos, vel, kp, kd, tau) of an MIT frame. Tau rides in the CAN ID."""
    m = MODELS[model]
    u = [int.from_bytes(data[i:i + 2], 'big') for i in (0, 2, 4, 6)]
    return (
        u16_to_float(u[0], P_MIN, P_MAX),
        u16_to_float(u[1], m.v_min, m.v_max),
        u16_to_float(u[2], 0.0, KP_MAX),
        u16_to_float(u[3], 0.0, KD_MAX),
        u16_to_float((can_id >> 8) & 0xFFFF, m.t_min, m.t_max),
    )


def encode_feedback(motor_id, host_id, pos, vel, tau, temp_c, mode, fault_bits, model):
    """Return (can_id, data) of a feedback frame. Temperature is unsigned tenths of a degree."""
    m = MODELS[model]
    temp = min(max(math.floor(temp_c * 10.0 + 0.5), 0), 0xFFFF)
    data = b''.join(
        v.to_bytes(2, 'big')
        for v in (
            float_to_u16(pos, P_MIN, P_MAX),
            float_to_u16(vel, m.v_min, m.v_max),
            float_to_u16(tau, m.t_min, m.t_max),
            temp,
        )
    )
    can_id = (
        (COMM_FEEDBACK << 24)
        | ((mode & 0x03) << 22)
        | ((fault_bits & 0x3F) << 16)
        | (motor_id << 8)
        | host_id
    )
    return can_id, data


def param_index(data):
    """Return the parameter index of a read or write frame (little-endian)."""
    return int.from_bytes(data[0:2], 'little')


def param_raw(data):
    """Return the 4 raw value bytes of a write frame, or of a read reply."""
    return bytes(data[4:8])


def encode_param_reply(motor_id, host_id, index, raw):
    """Return (can_id, data) answering a parameter read. The motor id rides in bits 15..8."""
    return (
        make_can_id(COMM_READ_PARAM, motor_id, host_id),
        index.to_bytes(2, 'little') + b'\0\0' + raw,
    )
