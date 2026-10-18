# maker_arm_ros2

ROS 2 Jazzy packages for the MakerMods Maker Arm. They hold the robot description, a ros2_control plugin, MoveIt and Servo, launch files, a MuJoCo plant and two demos.

The plugin talks to a daemon from maker-arm-rs (`maker-arm-cli serve`). The daemon owns the motors. If ROS stops, the daemon keeps holding the arm.

Status: the stack is verified in simulation. It is not yet verified on a real arm. The real-arm path needs a profile that you make from a calibration of your own arm. Read the real-arm section first.

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
    git -C maker-arm-rs checkout b2bae867ec099d3bf7bb650adae70ec04782e9d0
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
ros2 run maker_arm_demos replay /tmp/pick_place.json --raised 0.05
```

The replay also refuses a record whose fingertips come within 2 cm of the modeled desk. A pick and place record grasps closer than that. `--raised M` says that the arm stands M meters above the modeled desk. The replay then adds M to the fingertip height. M must be at least 0.05. In simulation the arm stays on the desk, so `--raised 0.05` only lets the record through. On the real arm, raise the arm at least 5 cm above the modeled desk before a replay. Then pass that height as `--raised`.

Before it asks for `yes`, the replay prints the duration, the joint ranges and the lowest fingertip height.

If you want the replay to move the arm to the start pose first, add `--approach`. It plans at scaling 0.1. It asks you to type `yes` twice, before the approach and before the replay.

## The real arm

You make the profile of your arm with `maker-arm-cli export`. The calibration measurement procedure is not published yet. A guessed calibration can drive a joint into its stop, so do not guess.

The defaults are the shipped `dynamics_urdf.json` and no params file. Later, the defaults become one identified model and its session 5 params. They always change as a pair. Never mix the dynamics file and the params file of different identifications.

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

5. Make the profile of your arm. The calibration file is JSON with two keys. `direction` has 7 values, each -1.0 or 1.0. `offset` has 7 values, each within pi. The command writes the profile with caps 0.5 of the nominal gains and torque caps. For a capped profile, the daemon also sets the error clamp.

    ```sh
    maker-arm-cli export --calibration CAL.json --caps 0.5 --out ~/maker_arm/real_profile.json
    ```

6. Launch the stack with that profile. Without `profile:=`, the launch stops and prints the export command.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py profile:=$HOME/maker_arm/real_profile.json rviz:=true
    ```

   If the launch stops with a message that names a file that does not exist, stop. Fix the path. Never mix a dynamics file and a params file of different identifications.
7. If the launch says that no daemon answered, read the log that the message names. To find the process that holds the CAN bus, run `ss -xlp | grep maker-arm-bus-can0`. A daemon can hold the arm with no socket. Support the arm before you stop that process, because the arm then drops.
8. Enabling the motors holds the measured pose. The daemon also enables a joint that rests up to 0.35 rad outside its soft limits. It then pulls that joint back inside the limits, so that joint can move by up to 0.35 rad. Keep the arm supported until the launch finishes.
9. In RViz, open the MotionPlanning panel. Set the velocity and acceleration scaling to 0.1. Plan and execute the named pose `home`.
10. Compare the real arm with RViz. If they differ, stop. Support the arm and release it. A bad zero or a wrong joint map causes this.
11. Only after the arm matches RViz, try other moves. The default scaling is 0.2.
12. Use nominal caps only after the arm matches RViz and finishes the moves of steps 9 and 11 at caps 0.5. Write the nominal profile to a second file, so the caps 0.5 profile stays.

    ```sh
    maker-arm-cli export --calibration CAL.json --caps 1.0 --out ~/maker_arm/real_profile_nominal.json
    ros2 launch maker_arm_bringup real.launch.py profile:=$HOME/maker_arm/real_profile_nominal.json rviz:=true
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
9. On the real arm, replay a record only with the arm raised at least 5 cm above the modeled desk. Pass that height as `--raised`.

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
