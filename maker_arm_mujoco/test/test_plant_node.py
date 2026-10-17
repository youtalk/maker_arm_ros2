import errno
import queue
import socket
import threading
import time

from maker_arm_mujoco import plant_node
from maker_arm_mujoco import robstride as rs
from maker_arm_mujoco.motors import MotorBank
import mujoco
import pytest
import rclpy


class Log:

    def __init__(self):
        self.lines = []

    def warning(self, msg, **_):
        self.lines.append(msg)

    error = warning


class Bus:
    """A CAN socket that delivers `frames`, then raises `recv_error`. Every send raises."""

    def __init__(self, frames, recv_error, send_error):
        self.frames, self.recv_error, self.send_error = list(frames), recv_error, send_error

    def recv(self, _):
        if self.frames:
            return self.frames.pop(0)
        raise self.recv_error

    def send(self, _):
        raise self.send_error


def enable_frame(motor):
    can_id = rs.make_can_id(rs.COMM_ENABLE, 0xFD, motor) | socket.CAN_EFF_FLAG
    return plant_node.CAN_FRAME.pack(can_id, 8, bytes(8))


def test_a_can_error_drops_the_reply_and_the_plant_keeps_stepping():
    bank, log = MotorBank(['RS00'] * 7), Log()
    full = OSError(errno.ENOBUFS, 'No buffer space available')
    down = OSError(errno.ENETDOWN, 'Network is down')
    bus = Bus([enable_frame(1), enable_frame(2)], down, full)
    assert plant_node.answer(bus, bank, 0.0, log) == 2  # both replies dropped, nothing raised
    assert bank.motors[0].enabled and bank.motors[1].enabled
    assert any('No buffer space' in line for line in log.lines)
    assert any('Network is down' in line for line in log.lines)
    quiet = Bus([], BlockingIOError(errno.EAGAIN, 'again'), full)
    assert plant_node.answer(quiet, bank, 0.0, Log()) == 0


def test_the_plant_fails_before_ready_without_a_renderer(monkeypatch, capsys):
    def no_gl(*_):
        raise mujoco.FatalError('gladLoadGL error')

    monkeypatch.setattr(plant_node, 'open_can', lambda interface: Bus([], BlockingIOError(), None))
    monkeypatch.setattr(mujoco, 'Renderer', no_gl)
    with pytest.raises(SystemExit, match='renderer'):
        plant_node.main()
    assert 'PLANT_READY' not in capsys.readouterr().out


def test_a_failed_camera_frame_is_logged_and_the_camera_keeps_going(monkeypatch):
    def bad_jpeg(*_):
        raise ValueError('no jpeg')

    monkeypatch.setattr(plant_node, 'open_can', lambda interface: Bus([], BlockingIOError(), None))
    monkeypatch.setattr(plant_node.cv2, 'imencode', bad_jpeg)
    rclpy.init()
    try:
        node = plant_node.PlantNode()
        node.camera_hz = 100.0
        log = Log()
        node.get_logger = lambda: log
        started = queue.Queue(1)
        camera = threading.Thread(target=node.camera_loop, args=(started,), daemon=True)
        camera.start()
        assert started.get(timeout=30) is None
        deadline = time.monotonic() + 10
        while len(log.lines) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(log.lines) >= 3  # the thread lived past its first failed frame
        assert 'no jpeg' in log.lines[0]
    finally:
        rclpy.try_shutdown()
    camera.join(timeout=5)
    assert not camera.is_alive()
