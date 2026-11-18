# Run a Conduit demo alone

These steps are for one person who runs a demo session with Conduit on the real arm. They are a short form of [The real arm](../README.md#the-real-arm). On a new computer or a new arm, do the full steps there first.

Conduit connects through Zenoh, so install it once.

```sh
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

Each terminal must use the same RMW and domain ID as Conduit. Run these lines in each new terminal. The domain ID 123 is an example.

```sh
source /opt/ros/jazzy/setup.bash
source ~/maker_arm_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=123
```

1. Turn on the motor power supply and release the emergency stop. If `can0` is down, bring it up (step 2 of [The real arm](../README.md#the-real-arm)).
2. In terminal 1, start the Zenoh router. Keep it running for the whole session.

    ```sh
    ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

3. In terminal 2, make sure that no daemon runs and that all seven motors answer.

    ```sh
    maker-arm-cli status --interface can0
    maker-arm-cli --can can0 scan
    ```

    If `status` prints a `STATUS` line, a daemon runs. If its mode is `released`, end it (see [End a released daemon](../README.md#end-a-released-daemon)). `scan` must show motors 1 to 7 with `present` `true` and `no fault`.

4. Fold the arm into its rest and support it. Keep a hand near the emergency stop, not on it. Then, in terminal 2, launch the stack with both cameras. The launch enables the motors.

    ```sh
    ros2 launch maker_arm_bringup real.launch.py rviz:=true \
        wrist_camera:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0 \
        head_camera:=/dev/v4l/by-id/usb-Remo_Tech_Co.__Ltd._OBSBOT_Meet_2-video-index0
    ```

5. Keep the arm supported. In terminal 3, move the arm to `home`. It prints `go_home: the arm is at home and the jaw is closed`.

    ```sh
    ros2 run maker_arm_bringup go_home
    ```

6. In Conduit, tap the gear icon, set these values and connect.

    | Item | Value |
    | --- | --- |
    | Transport | Zenoh |
    | Router Address | the IP address of this computer |
    | Router Port | 7447 |
    | Domain ID | the `ROS_DOMAIN_ID` above |
    | Distribution | Jazzy |

7. Start the take within 60 s of `go_home`. Move to `work` with a Conduit pose, or press HOLD and move the phone. If the arm stays at `home` for 60 s, it parks. Then wake it (step 9).
8. During the take, remember these rules:

    - If nobody drives the arm for 30 s and the arm is away from `home`, it returns to `home` and parks. Put a held object down before you stop, because the park turns off the gripper motor.
    - If Conduit loses Wi-Fi during HOLD, Servo keeps the last command for up to 0.5 s. Then it stops the arm.
    - A pose move runs on the computer. If Conduit loses Wi-Fi during a pose move, the move continues to its end.

9. Between takes, let the arm park and cool. J3 heats about 2.5 degrees C per minute while it holds a pose. Run `maker-arm-cli status --interface can0` and read `temp_max_c`. When it is below 40, wake the arm: support it, run `wake` in terminal 3 and type `ENABLE`. `wake` ends with `go_home`, so start the next take within 60 s (step 7).

    ```sh
    ros2 run maker_arm_bringup wake
    ```

10. To end the session, support the arm. Let it park, or run `ros2 run maker_arm_bringup release` and type `RELEASE`. Stop the launch with Ctrl-C in terminal 2. End the released daemon (see [End a released daemon](../README.md#end-a-released-daemon)). Stop the router with Ctrl-C in terminal 1. Press the emergency stop and turn off the power supply.

If the motors lose power, every joint loses its torque at the same time and the arm drops. The launch log shows a motor fault and feedback timeouts. Do these steps:

1. Support the arm and press the emergency stop.
2. Run `ros2 run maker_arm_bringup release` and type `RELEASE`. The motors do not answer the disable, so the daemon ends.
3. Stop the launch with Ctrl-C.
4. Disconnect the AC cord of the motor power supply for 10 s or more. Connect it again and release the emergency stop. After a power loss, the supply can stay in a loop that turns it off and on every 2.5 s. An emergency stop cycle does not stop this loop. Only the AC cycle stops it.
5. Run `maker-arm-cli --can can0 scan`. If all seven motors answer, start again from step 4 of the session steps above.
