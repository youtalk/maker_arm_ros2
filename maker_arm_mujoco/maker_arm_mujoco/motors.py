"""Seven RobStride motors on a CAN bus, replying as maker-arm-rs's SimArm does."""

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
    """Motor ids are 1..len(models), in joint order."""

    def __init__(self, models: Sequence[str], host_id: int = 0xFD):
        self.host_id = host_id
        self.motors = [_Motor(m) for m in models]

    def _disable(self, m):
        m.enabled = False
        m.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)

    def _reply(self, i):
        m = self.motors[i]
        return rs.encode_feedback(
            i + 1, self.host_id, m.q, m.dq, m.tau, m.temp_c, 2 if m.enabled else 0, 0, m.model
        )

    def feedback(self, i, q, dq, tau, temp_c=AMBIENT_C):
        """Set the state that the next replies report (motor frame)."""
        m = self.motors[i]
        m.q, m.dq, m.tau, m.temp_c = q, dq, tau, temp_c

    def handle(self, can_id: int, data: bytes, now_s: float) -> list[tuple[int, bytes]]:
        """Take one frame from the host and return the frames that the motor sends back."""
        comm, _, target = rs.split_can_id(can_id)
        i = target - 1
        if not 0 <= i < len(self.motors):
            return []
        m = self.motors[i]
        m.last_frame_s = now_s
        if comm == rs.COMM_ENABLE:
            m.enabled = True
        elif comm == rs.COMM_DISABLE:  # also the read-only probe
            self._disable(m)
        elif comm == rs.COMM_MIT:
            if m.enabled:
                m.cmd = rs.decode_mit(can_id, data, m.model)
        elif comm == rs.COMM_SET_ZERO:
            # ponytail: replies zero, the plant keeps its own zero; add an offset if needed
            return [rs.encode_feedback(
                target, self.host_id, 0.0, m.dq, m.tau, m.temp_c, 2 if m.enabled else 0, 0, m.model
            )]
        elif comm == rs.COMM_READ_PARAM:
            index = rs.param_index(data)
            if index not in m.params:
                return []
            return [rs.encode_param_reply(target, self.host_id, index, m.params[index])]
        elif comm == rs.COMM_WRITE_PARAM:  # fire and forget
            m.params[rs.param_index(data)] = rs.param_raw(data)
            return []
        else:
            return []
        return [self._reply(i)]

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
