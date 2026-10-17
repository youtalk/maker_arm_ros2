# maker_arm_ros2

ROS 2 Jazzy packages for the MakerMods Maker Arm. They hold the robot description, a ros2_control plugin, MoveIt and Servo, launch files, a MuJoCo plant and two demos.

The plugin talks to a daemon from maker-arm-rs (`maker-arm-cli serve`). The daemon owns the motors. If ROS stops, the daemon keeps holding the arm.

Status: the stack is verified in simulation. It is not yet verified on a real arm. The real-arm files ship after the MS5 sessions, so the real-arm path is not ready. Do not run `real.launch.py` with hand-made profile, dynamics or params files.

## Install

These steps are for Ubuntu 24.04 with ROS 2 Jazzy.

1. Install the ROS packages.

    ```sh
    sudo apt install ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-moveit-servo \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers can-utils
    ```

2. Install the daemon. Ubuntu's default `cargo` is too old, so use the versioned packages.

    ```sh
    sudo apt install rustc-1.85 cargo-1.85 git
    git clone https://github.com/youtalk/maker-arm-rs
    git -C maker-arm-rs checkout 31b3584b195bf225d71e5bffdec04c073a640332
    PATH=/usr/lib/rust-1.85/bin:$PATH cargo install --locked --path maker-arm-rs/crates/maker-arm-cli
    ```

3. Add `~/.cargo/bin` to your `PATH`. The plugin starts `maker-arm-cli` from there.

4. Clone this repository into a workspace.

    ```sh
    mkdir -p ~/maker_arm_ws/src
    git clone https://github.com/youtalk/maker_arm_ros2 ~/maker_arm_ws/src/maker_arm_ros2
    cd ~/maker_arm_ws
    ```

5. Make a venv for MuJoCo. `mujoco` is not a rosdep key, so pip installs it.

    ```sh
    python3 -m venv --system-site-packages .venv
    .venv/bin/pip install mujoco==3.13.0
    ```

6. Build with the venv's Python.

    ```sh
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths src --ignore-src -y
    .venv/bin/python -m colcon build
    source install/setup.bash
    ```

## Simulation

Start here. No arm and no CAN hardware are needed.

### SimArm

SimArm is the daemon's built-in simulated arm. Run it first.

```sh
ros2 launch maker_arm_bringup simarm.launch.py rviz:=true
```

### MuJoCo on vcan0

The MuJoCo plant answers on a virtual CAN bus as the arm's seven motors. The daemon and the stack run as on the real arm.

1. Bring up `vcan0`. This needs sudo once per boot.

    ```sh
    sudo bash src/maker_arm_ros2/maker_arm_mujoco/scripts/vcan_up.sh
    ```

2. Launch the plant and the stack. The stack starts after the plant prints `PLANT_READY`.

    ```sh
    ros2 launch maker_arm_bringup sim.launch.py rviz:=true
    ```

The plant publishes the wrist camera on `/wrist_camera/image_raw/compressed`. Launch arguments are `scene` (a scene file) and `seed` (the scene perturbation).

### Demos

Run the pick and place demo with the simulation running. It writes a record of the moves to `--out`.

```sh
ros2 run maker_arm_demos pick_place --seed 1 --out /tmp/pick_place.json
```

To replay a record at half speed, run the next command. If the arm is not at the record's start pose, the replay refuses to start.

```sh
ros2 run maker_arm_demos replay /tmp/pick_place.json
```

If you want the replay to move the arm to the start pose first, add `--approach`. It plans at scaling 0.1. It asks you to type `yes` twice, before the approach and before the replay.

## The real arm

The real-arm path is not ready. The default files of `real.launch.py` ship after the MS5 sessions. Until then, the launch stops and names the missing file. Do not run it with hand-made profile, dynamics or params files. The steps below are for later, after the shipped files exist.

If the real arm loses power or CAN, it can fall and break. Software cannot prevent this. Follow the steps in order.

1. Keep your runtime directory after you log out. Do this once on each computer.

    ```sh
    loginctl enable-linger $USER
    ```

    When your last login session ends, logind removes `$XDG_RUNTIME_DIR`. The daemon's socket is in that directory. Without the socket, the daemon keeps holding the arm, but no launch and no `release` can reach it.

2. Support the arm with your hand or a stand. Do this before the first launch and before every release.
3. Ask the daemon for its status. A daemon can already hold the arm.

    ```sh
    maker-arm-cli status --interface can0
    ```

    If it prints a `STATUS` line, a daemon holds the arm. Do not run `doctor`. Go to step 5. In that case `doctor` refuses anyway, because the daemon holds the bus lock.

4. Run the daemon's check on the CAN interface. If it reports a problem, stop here.

    ```sh
    maker-arm-cli --can can0 doctor
    ```

5. Launch the stack. By default it starts at caps 0.5 of the nominal gains and torque caps.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py rviz:=true
    ```

6. If the launch stops with a message that names a missing file, stop. The shipped files are not installed. The dynamics file and the params file ship as a pair from the same identification. Never mix files from different identifications.
7. If the launch says that no daemon answered, read the log that the message names. To find the process that holds the CAN bus, run `ss -xlp | grep maker-arm-bus-can0`. A daemon can hold the arm with no socket. Support the arm before you stop that process, because the arm then drops.
8. Enabling the motors holds the measured pose. The daemon also enables a joint that rests up to 0.35 rad outside its soft limits. It then pulls that joint back inside the limits, so that joint can move by up to 0.35 rad. Keep the arm supported until the launch finishes.
9. In RViz, open the MotionPlanning panel. Set the velocity and acceleration scaling to 0.1. Plan and execute the named pose `home`.
10. Compare the real arm with RViz. If they differ, stop. Support the arm and release it. A bad zero or a wrong joint map causes this.
11. Only after the arm matches RViz, try other moves. The default scaling is 0.2.
12. To use nominal caps, launch with the nominal profile. The file ships with the others after the MS5 sessions.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py \
        profile:=$(ros2 pkg prefix maker_arm_description)/share/maker_arm_description/config/daemon/real_profile_nominal.json
    ```

13. At the end of the session, support the arm. Then release it.

    ```sh
    ros2 run maker_arm_bringup release
    ```

    Only a typed `RELEASE` disables the motors.

If you stop the launch with Ctrl-C, the daemon keeps holding the arm. To turn the motors off, run the release step.

## Conduit

Use these values to connect Conduit.

| Item | Value |
| --- | --- |
| Base frame | `mount_link` |
| Gripper action | `/gripper_controller/gripper_cmd` |
| Gripper open | 0.050636 m per jaw |
| Gripper closed | 0.0017765 m per jaw |
| Wrist camera (simulation only) | `/wrist_camera/image_raw/compressed` |

Servo halts near the `ready` pose because of a singularity. Before you teleoperate, move away from `ready`. Use the `home` pose or a MoveIt goal.

## Safety rules

1. Support the arm before the first launch and before every release.
2. Before the first launch, run `maker-arm-cli status --interface can0`. If it prints no `STATUS` line, run `maker-arm-cli --can can0 doctor`.
3. Start at caps 0.5. Move to `home` at scaling 0.1 and compare the arm with RViz before any other motion.
4. Do not release the arm while it is unsupported. Release only through `ros2 run maker_arm_bringup release`.
5. Do not kill the daemon and do not cut power or CAN while the motors are on. The arm drops.
6. Keep a person at the e-stop on the first real runs.
7. Do not run the stack with files from another arm or with hand-made files. Never mix the dynamics file and the params file of different identifications. If a joint is more than 0.35 rad outside its limits, the daemon refuses to enable. This catches some bad zeros, not all.
8. If the plugin refuses a running daemon, support the arm and run the release step. Then launch again.

What the stack does for you:

- If ROS crashes or stops sending commands, the daemon holds the arm.
- The daemon refuses a command jump and clamps every command to the profile.
- Enable holds the measured pose, or the nearest pose inside the joint limits.
- MoveIt keeps a margin inside the joint limits. The collision matrix and a desk object protect the arm and the desk.
- The gripper squeeze torque is limited.

## Packages

- `maker_arm_description`: URDF, meshes and the daemon model files.
- `maker_arm_hardware`: the ros2_control plugin.
- `maker_arm_moveit_config`: MoveIt and Servo configuration.
- `maker_arm_bringup`: launch files, controllers, RViz and the release command.
- `maker_arm_mujoco`: the MuJoCo plant.
- `maker_arm_demos`: pick and place, replay and test tools.

## License

Apache-2.0.
