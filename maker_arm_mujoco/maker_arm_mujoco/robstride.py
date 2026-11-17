"""RobStride MIT CAN protocol, the motor side. Mirrors maker-arm-protocol (Rust)."""

from typing import NamedTuple

HOST_ID = 0xFD  # the id the motors reply on
P_MIN, P_MAX = -12.57, 12.57
KP_MAX = 500.0
KD_MAX = 5.0

PARAM_READ_ID = 0x300
PARAM_WRITE_ID = 0x400

OP_CLEAR_FAULT = 0xFB
OP_ENABLE = 0xFC
OP_DISABLE = 0xFD
OP_SET_ZERO = 0xFE

RUN_MODE = 0x7005  # u8
VBUS = 0x701C  # f32
CAN_TIMEOUT = 0x7028  # u32, 50 us per count
CAN_TIMEOUT_PER_MS = 20


class Model(NamedTuple):
    """The mapping ranges of one motor model."""

    t_max: float
    v_max: float

    @property
    def t_min(self):
        return -self.t_max

    @property
    def v_min(self):
        return -self.v_max


MODELS = {'RS00': Model(14.0, 33.0), 'RS02': Model(17.0, 44.0)}


def float_to_uint(x, lo, hi, bits):
    """Clamp and scale to 0..2^bits-1. Python's round is half to even, like the Rust codec."""
    return round((min(max(x, lo), hi) - lo) * ((1 << bits) - 1) / (hi - lo))


def uint_to_float(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


def decode_control(data, model):
    """Return (pos, vel, kp, kd, tau) of a control payload, or None for a command payload."""
    if bytes(data[:6]) == b'\xff' * 6:
        return None
    m = MODELS[model]
    q = data[0] << 8 | data[1]
    v = data[2] << 4 | data[3] >> 4
    k = (data[3] & 0xF) << 8 | data[4]
    d = data[5] << 4 | data[6] >> 4
    t = (data[6] & 0xF) << 8 | data[7]
    return (
        uint_to_float(q, P_MIN, P_MAX, 16),
        uint_to_float(v, m.v_min, m.v_max, 12),
        uint_to_float(k, 0.0, KP_MAX, 12),
        uint_to_float(d, 0.0, KD_MAX, 12),
        uint_to_float(t, m.t_min, m.t_max, 12),
    )


def encode_feedback(motor_id, pos, vel, tau, temp_c, mode, fault, warning, legacy, model='RS00'):
    """
    Return the 8 bytes of a feedback reply on HOST_ID.

    Byte 6 holds mode, fault and warning in its high nibble and the temperature's top bits in
    its low nibble. A legacy motor sends the high nibble as 0.
    """
    m = MODELS[model]
    q = float_to_uint(pos, P_MIN, P_MAX, 16)
    v = float_to_uint(vel, m.v_min, m.v_max, 12)
    t = float_to_uint(tau, m.t_min, m.t_max, 12)
    temp = min(max(round(temp_c * 10.0), 0), 0xFFF)
    status = 0 if legacy else mode << 6 | fault << 5 | warning << 4
    return bytes([
        motor_id, q >> 8, q & 0xFF, v >> 4, (v & 0xF) << 4 | t >> 8, t & 0xFF,
        status | temp >> 8, temp & 0xFF,
    ])


def encode_fault_status(motor_id, word):
    """Return the 5 bytes of a fault status reply: the motor id and the fault word."""
    return bytes([motor_id]) + word.to_bytes(4, 'little')


def param_index(data):
    """Return the parameter index of a read or write frame (little-endian)."""
    return int.from_bytes(data[0:2], 'little')


def param_raw(data):
    """Return the 4 raw value bytes of a write frame."""
    return bytes(data[4:8])


def encode_param_reply(motor_id, index, raw, write):
    """Return (can_id, data) answering a parameter read or write."""
    base = PARAM_WRITE_ID if write else PARAM_READ_ID
    return base | motor_id, index.to_bytes(2, 'little') + b'\0\0' + raw
