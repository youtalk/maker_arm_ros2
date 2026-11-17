"""Seven RobStride motors on a CAN bus, replying as maker-arm-rs's SimArm does (MIT protocol)."""

from typing import Sequence

from . import robstride as rs

AMBIENT_C = 35.0


class _Motor:
    def __init__(self, model):
        self.model = model
        self.enabled = False
        self.last_frame_s = None
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)  # pos, vel, kp, kd, tau
        self.q = self.dq = self.tau = 0.0
        self.temp_c = AMBIENT_C
        self.fault_word = 0
        # Parameter store, seeded like SimArm: RUN_MODE 0, CAN_TIMEOUT 0 (no watchdog), VBUS 24 V.
        self.params = {
            rs.RUN_MODE: bytes(4),
            rs.CAN_TIMEOUT: bytes(4),
            rs.VBUS: bytes.fromhex('0000C041'),
        }

    def watchdog_s(self):
        counts = int.from_bytes(self.params[rs.CAN_TIMEOUT], 'little')
        return counts / (rs.CAN_TIMEOUT_PER_MS * 1000.0)


class MotorBank:
    """
    Motor ids are 1..len(models), in joint order. Frames carry standard ids only.

    By default the bank acts like the owner's firmware: no parameter access, and the mode and
    flag bits of a reply stay 0. With `params_supported` it answers parameter frames, reports
    mode and flags, and runs the CAN_TIMEOUT watchdog.
    """

    def __init__(self, models: Sequence[str], host_id: int = rs.HOST_ID, params_supported=False):
        self.host_id = host_id
        self.params_supported = params_supported
        self.motors = [_Motor(m) for m in models]

    def _disable(self, m):
        m.enabled = False
        m.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)

    def _feedback(self, i):
        m = self.motors[i]
        p = self.params_supported
        data = rs.encode_feedback(
            i + 1, m.q, m.dq, m.tau, m.temp_c, 2 if m.enabled and p else 0,
            p and m.fault_word != 0, False, not p, m.model)
        return self.host_id, data

    def feedback(self, i, q, dq, tau, temp_c=AMBIENT_C):
        """Set the state that the next replies report (motor frame)."""
        m = self.motors[i]
        m.q, m.dq, m.tau, m.temp_c = q, dq, tau, temp_c

    def _command(self, i, arg, op):
        m = self.motors[i]
        if op == rs.OP_CLEAR_FAULT and arg == 0x00:  # the fault query does not clear
            return [(self.host_id, rs.encode_fault_status(i + 1, m.fault_word))]
        if op == rs.OP_CLEAR_FAULT:
            m.fault_word = 0
        elif op == rs.OP_ENABLE:
            m.enabled = True
        elif op == rs.OP_DISABLE:
            self._disable(m)
        elif op == rs.OP_SET_ZERO:
            m.q = 0.0  # ponytail: the plant keeps its own zero; add an offset if needed
        else:
            return []
        return [self._feedback(i)]

    def _param(self, can_id, i, data):
        m = self.motors[i]
        write = can_id & 0x780 == rs.PARAM_WRITE_ID
        index = rs.param_index(data)
        if write:
            m.params[index] = rs.param_raw(data)
        if index not in m.params:
            return []
        return [rs.encode_param_reply(i + 1, index, m.params[index], write)]

    def handle(self, can_id: int, data: bytes, now_s: float) -> list[tuple[int, bytes]]:
        """Take one frame from the host and return the (id, data) frames the motor sends back."""
        i = (can_id & 0x7F) - 1
        if len(data) != 8 or can_id > 0x7FF or not 0 <= i < len(self.motors):
            return []
        m = self.motors[i]
        m.last_frame_s = now_s
        kind = can_id & 0x780
        if kind == 0:
            control = rs.decode_control(data, m.model)
            if control is None:
                return self._command(i, data[6], data[7])
            if m.enabled:
                m.cmd = control
            return [self._feedback(i)]
        if self.params_supported and kind in (rs.PARAM_READ_ID, rs.PARAM_WRITE_ID):
            return self._param(can_id, i, data)
        return []

    def torques(self, q: Sequence[float], dq: Sequence[float], now_s: float) -> list[float]:
        """Return the MIT law per motor, saturated at the model's t_max. Zero when disabled."""
        out = []
        for m, qi, dqi in zip(self.motors, q, dq):
            wd = m.watchdog_s()
            if m.enabled and wd > 0 and now_s - m.last_frame_s > wd:
                self._disable(m)
            if not m.enabled:
                out.append(0.0)
                continue
            p, v, kp, kd, tau = m.cmd
            t_max = rs.MODELS[m.model].t_max
            out.append(min(max(kp * (p - qi) + kd * (v - dqi) + tau, -t_max), t_max))
        return out
