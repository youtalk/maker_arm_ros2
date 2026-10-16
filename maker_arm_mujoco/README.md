# maker_arm_mujoco

A MuJoCo plant for the Maker Arm. It answers on the virtual CAN bus `vcan0` as the arm's seven RobStride motors. The maker-arm-rs daemon and the ROS stack above it run unchanged.

## Install

`mujoco` is not a rosdep key. Install it with pip into a venv that sees the ROS packages, and build with that venv's Python:

    python3 -m venv --system-site-packages .venv
    .venv/bin/pip install mujoco==3.13.0

## Run

1. Bring up `vcan0` once per boot: `sudo bash maker_arm_mujoco/scripts/vcan_up.sh`.
2. Start the plant: `ros2 run maker_arm_mujoco plant`. After it starts to answer on the bus, it prints `PLANT_READY`.

Parameters: `interface` (`vcan0`), `scene` (`config/scene.yaml`), `seed` (0, the seed of `scene.perturbed`) and `camera_hz` (15).

Topics: `/wrist_camera/image_raw/compressed`, `/wrist_camera/camera_info`, and the TF frames `object/<name>` in `mount_link`. The plant renders with EGL unless `MUJOCO_GL` names another backend.

Until the daemon first enables a motor, the plant holds the arm at the ready pose, as a user supports the real arm. Every 10 s it logs how many physics steps started more than 2 ms late.
