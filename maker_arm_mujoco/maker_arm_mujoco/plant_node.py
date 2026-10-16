"""
The MuJoCo Maker Arm on vcan0, answering the daemon as its seven RobStride motors.

The main thread runs physics at 1 kHz on the wall clock. Each step drains the CAN frames into
the motor bank, sends the replies and steps the model under the motors' MIT law. A camera
thread renders the wrist camera from a copy of the joint state and publishes the object poses.
"""

import math
from pathlib import Path
import socket
import struct
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import TransformStamped
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .model import build_model, description, IMAGE_SIZE, PHYSICS_DT, Plant, ready, sim_profile
from .scene import load_scene, perturbed

CAN_FRAME = struct.Struct('=IB3x8s')  # struct can_frame: id, length, padding, data
LATE_S = 0.002
LOG_EVERY_S = 10.0
CAMERA_FRAME = 'wrist_camera_optical_frame'
SHARE = Path(get_package_share_directory('maker_arm_mujoco'))


def open_can(interface):
    """Open a raw, nonblocking socket on a virtual CAN bus. The plant never joins a real one."""
    if not interface.startswith('vcan'):
        raise SystemExit(f'the plant runs only on a virtual CAN bus (vcan*), not on {interface}')
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
    except OSError as e:
        sock.close()
        raise SystemExit(
            f'cannot open CAN interface {interface}: {e.strerror}. Bring vcan0 up once with: '
            f'sudo bash {SHARE / "scripts" / "vcan_up.sh"}'
        ) from None
    sock.setblocking(False)
    return sock


def transform(stamp, parent, child, pos, quat_wxyz):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    v, r = t.transform.translation, t.transform.rotation
    v.x, v.y, v.z = (float(x) for x in pos)
    r.w, r.x, r.y, r.z = (float(x) for x in quat_wxyz)
    return t


class PlantNode(Node):

    def __init__(self):
        super().__init__('mujoco_plant')
        interface = self.declare_parameter('interface', 'vcan0').value
        scene = self.declare_parameter('scene', str(SHARE / 'config' / 'scene.yaml')).value
        seed = self.declare_parameter('seed', 0).value
        self.camera_hz = self.declare_parameter('camera_hz', 15.0).value
        self.sock = open_can(interface)
        self.scene = perturbed(load_scene(scene), seed)
        profile = sim_profile()
        self.plant = Plant(build_model(*description(), self.scene), profile, ready(profile))
        self.lock = threading.Lock()
        self.image_pub = self.create_publisher(
            CompressedImage, '/wrist_camera/image_raw/compressed', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/wrist_camera/camera_info', 1)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        cam = self.plant.model.camera('wrist_camera')
        optical = np.zeros(4)
        mujoco.mju_mulQuat(optical, cam.quat, np.array([0.0, 1.0, 0.0, 0.0]))  # back to ROS axes
        self.static_tf.sendTransform(
            transform(self.get_clock().now().to_msg(), 'link_007', CAMERA_FRAME, cam.pos, optical))

    def camera_info(self):
        f = IMAGE_SIZE / 2 / math.tan(math.radians(self.plant.model.cam_fovy[0]) / 2)
        c = IMAGE_SIZE / 2
        info = CameraInfo(width=IMAGE_SIZE, height=IMAGE_SIZE, distortion_model='plumb_bob')
        info.header.frame_id = CAMERA_FRAME
        info.d = [0.0] * 5
        info.k = [f, 0.0, c, 0.0, f, c, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, c, 0.0, 0.0, f, c, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def camera_loop(self):
        """Publish the wrist image and the object poses at camera_hz, from a copy of qpos."""
        model = self.plant.model
        view = mujoco.MjData(model)
        # The renderer's GL context belongs to the thread that makes it: this one.
        renderer = mujoco.Renderer(model, IMAGE_SIZE, IMAGE_SIZE)
        bodies = {o['name']: model.body(f'object/{o["name"]}').id for o in self.scene['objects']}
        info = self.camera_info()
        image = CompressedImage(format='jpeg')
        image.header.frame_id = CAMERA_FRAME
        due = time.monotonic()
        while rclpy.ok():
            with self.lock:
                view.qpos[:] = self.plant.data.qpos
            mujoco.mj_kinematics(model, view)
            mujoco.mj_camlight(model, view)
            stamp = self.get_clock().now().to_msg()
            self.tf.sendTransform([
                transform(stamp, 'mount_link', f'object/{name}', view.xpos[b], view.xquat[b])
                for name, b in bodies.items()
            ])
            renderer.update_scene(view, camera='wrist_camera')
            image.header.stamp = info.header.stamp = stamp
            image.data = cv2.imencode('.jpg', renderer.render()[:, :, ::-1])[1].tobytes()
            self.image_pub.publish(image)
            self.info_pub.publish(info)
            due += 1.0 / self.camera_hz
            time.sleep(max(0.0, due - time.monotonic()))

    def run(self):
        """Step physics at 1 kHz on the wall clock and answer the CAN frames between steps."""
        plant, sock = self.plant, self.sock
        start = time.monotonic()
        late = dropped = k = 0
        next_log = LOG_EVERY_S
        while rclpy.ok():
            now_s = k * PHYSICS_DT
            lag = time.monotonic() - start - now_s
            if lag < 0.0:
                time.sleep(-lag)
            elif lag > LATE_S:
                late += 1
            while True:
                try:
                    can_id, _, data = CAN_FRAME.unpack(sock.recv(CAN_FRAME.size))
                except BlockingIOError:
                    break
                if not can_id & socket.CAN_EFF_FLAG:
                    continue  # RobStride frames carry 29-bit ids
                for rid, rdata in plant.bank.handle(can_id & socket.CAN_EFF_MASK, data, now_s):
                    try:
                        sock.send(CAN_FRAME.pack(rid | socket.CAN_EFF_FLAG, len(rdata), rdata))
                    except BlockingIOError:
                        dropped += 1
            with self.lock:
                plant.step(now_s)
            if k == 0:
                print('PLANT_READY', flush=True)
            k += 1
            if now_s >= next_log:
                self.get_logger().info(
                    f'{late} steps more than {LATE_S * 1e3:.0f} ms late, '
                    f'{dropped} replies dropped')
                next_log += LOG_EVERY_S


def main():
    rclpy.init()
    try:
        node = PlantNode()
        threading.Thread(target=node.camera_loop, daemon=True).start()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
