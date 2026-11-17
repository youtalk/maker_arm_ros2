# maker_arm_ros2

ROS 2 Jazzy packages for the MakerMods Maker Arm. They hold the robot description, a ros2_control plugin, MoveIt and Servo, launch files, a MuJoCo plant and two demos.

The plugin talks to a daemon from maker-arm-rs (`maker-arm-cli serve`). The daemon owns the motors. If ROS stops, the daemon keeps holding the arm.

The daemon speaks the RobStride MIT protocol, as the vendor SDK and MakerModsLab do. The joint angles start from the vendor zero: the arm folded against the base, with the gripper fully closed.

Status: the stack is verified in simulation. It is not yet verified on a real arm. Read the real-arm section first.

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
    git -C maker-arm-rs checkout 3cf672aaa5f01d83dde6154fd42ef50ab6747772
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

The plant reports motor angles as the real arm does, with motor 5 turned against the URDF joint. So the daemon loads the shipped `real_profile_nominal.json`, and the calibration runs in simulation before it runs on the arm.

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

The shipped profiles are for an arm in the vendor zero. An arm zeroed in MakerModsLab or with the vendor SDK is in the vendor zero, and it needs no calibration file. `maker-arm-cli export` made both profiles from `config/daemon/vendor_calibration.json`:

- `real_profile_caps05.json`: caps 0.5 of the nominal gains and torque caps. It is the default of `real.launch.py`. For a capped profile, the daemon also sets the error clamp.
- `real_profile_nominal.json`: the nominal gains and torque caps.

For an arm with another zero, you make the profile yourself. The calibration file is JSON with two keys. `direction` has 7 values, each -1.0 or 1.0. `offset` has 7 values, each within pi. The measurement procedure is not published yet. A guessed calibration can drive a joint into its stop, so do not guess.

```sh
maker-arm-cli export --calibration CAL.json --caps 0.5 --out ~/maker_arm/real_profile.json
```

The defaults are the shipped `dynamics_urdf.json` and no params file. Later, the defaults become one identified model and its session 5 params. They always change as a pair. Never mix the dynamics file and the params file of different identifications.

If the real arm loses power or CAN, it can fall and break. Software cannot prevent this. Follow the steps in order.

1. Keep your runtime directory after you log out. Do this once on each computer.

    ```sh
    loginctl enable-linger $USER
    ```

    When your last login session ends, logind removes `$XDG_RUNTIME_DIR`. The daemon's socket is in that directory. Without the socket, the daemon keeps holding the arm, but no launch and no `release` can reach it.

2. Bring up `can0` at 1 Mbit/s. For a native CAN adapter, run this command.

    ```sh
    sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set up can0
    ```

    `restart-ms` brings the interface back after a bus-off. A bus-off can come, for example, after frames that no powered motor acknowledged.

3. Support the arm with your hand or a stand. Do this before the first launch and before every release.
4. Ask the daemon for its status. A daemon can already hold the arm.

    ```sh
    maker-arm-cli status --interface can0
    ```

    If it prints a `STATUS` line, a daemon holds the arm. Do not run `doctor`. Go to step 7. In that case `doctor` refuses anyway, because the daemon holds the bus lock.

5. Run the daemon's check on the CAN interface. If it reports a problem, stop here.

    ```sh
    maker-arm-cli --can can0 doctor
    ```

6. Check the calibration with no torque. The motors stay off for this whole step. Fold the arm at rest and run `scan`.

    ```sh
    maker-arm-cli --can can0 scan
    ```

    J2 and J3 read near 0, and the closed gripper reads near 0. If they do not, the arm is not in the vendor zero. Stop, and do not launch. Then move one joint at a time by hand, and run `scan` again after each move. Only the value of that joint changes.

7. Launch the stack. Without `profile:=`, it loads the shipped caps 0.5 profile. For an arm with another zero, add `profile:=$HOME/maker_arm/real_profile.json`.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py rviz:=true
    ```

   If the launch stops with a message that names a file that does not exist, stop. Fix the path. Never mix a dynamics file and a params file of different identifications.
8. If the launch says that no daemon answered, read the log that the message names. To find the process that holds the CAN bus, run `ss -xlp | grep maker-arm-bus-can0`. A daemon can hold the arm with no socket. Support the arm before you stop that process, because the arm then drops.
9. Enabling the motors holds the measured pose. The daemon also enables a joint that rests up to 0.35 rad outside its soft limits. It then pulls that joint back inside the limits, so that joint can move by up to 0.35 rad. In the folded rest, J2 and J3 sit just outside their limits, so they move a few degrees. Keep the arm supported until the launch finishes.
10. Bring the arm into MoveIt's planning range. After enable, J2, J3 and the jaw rest just outside that range, and MoveIt refuses to plan from there. This command moves the arm to `home` in 5 s and closes the jaw, through the controllers and not through MoveIt. It checks no collisions, so run it only from the folded rest, right after enable. It refuses to move when a joint is more than 0.6 rad from `home`, for example from a raised pose.

    ```sh
    ros2 run maker_arm_bringup go_home
    ```

11. In RViz, open the MotionPlanning panel. Set the velocity and acceleration scaling to 0.1. Plan and execute the named pose `home`. It is the folded rest, pulled inside the limits.
12. Compare the real arm with RViz. If they differ, stop. Support the arm and release it. A bad zero or a wrong joint map causes this.
13. Only after the arm matches RViz, try other moves. The default scaling is 0.2.
14. Use nominal caps only after the arm matches RViz and finishes the moves of steps 11 and 13 at caps 0.5.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py rviz:=true \
        profile:=$(ros2 pkg prefix maker_arm_description)/share/maker_arm_description/config/daemon/real_profile_nominal.json
    ```

    For an arm with another zero, export a second file with `--caps 1.0`, so the caps 0.5 profile stays.

15. At the end of the session, support the arm. Then release it.

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
| Servo | the default names, `/servo_node/...` |
| Joint states | `/joint_states` |
| Gripper action | `/gripper_controller/gripper_cmd` |
| Gripper joint | `gripper_left_joint` |
| Gripper open | 0.0405171 m per jaw |
| Gripper closed | 0.0030216 m per jaw |
| Wrist camera (simulation only) | `/wrist_camera/image_raw/compressed` |

The open and closed values come from the vendor's gripper preview map. Measure them on your arm before you rely on them.

On the real arm, run `ros2 run maker_arm_bringup go_home` once after the launch, before you use Conduit's poses. Conduit's poses go through MoveIt, and MoveIt cannot plan from the rest pose after enable.

Before you teleoperate, move to `work` or `stand`. Servo halts at `home`, because `home` is a singularity. At `ready`, J4 sits near its limit, so Servo stops a push that turns J4 further down. Servo moves along every axis from `work` and `stand`, but it slows near them: in the simulation, a 1 s push delivered about 20 to 100% of the commanded motion. The real arm is not yet verified.

## Safety rules

1. Support the arm before the first launch and before every release.
2. Before the first launch, run `maker-arm-cli status --interface can0`. If it prints no `STATUS` line, run `maker-arm-cli --can can0 doctor`. Then check the calibration with no torque (step 6 of the real-arm section).
3. Start at caps 0.5. Move to `home` at scaling 0.1 and compare the arm with RViz before any other motion.
4. Do not release the arm while it is unsupported. Release only through `ros2 run maker_arm_bringup release`.
5. Do not kill the daemon and do not cut power or CAN while the motors are on. The arm drops.
6. Keep a person at the e-stop on the first real runs.
7. Do not run the stack with a profile made for another zero, or with hand-made files. Never mix the dynamics file and the params file of different identifications. If a joint is more than 0.35 rad outside its limits, the daemon refuses to enable. This catches some bad zeros, not all.
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
- `maker_arm_bringup`: launch files, controllers, RViz, and the release and go_home commands.
- `maker_arm_mujoco`: the MuJoCo plant.
- `maker_arm_demos`: pick and place, replay and test tools.

## License

Apache-2.0.
