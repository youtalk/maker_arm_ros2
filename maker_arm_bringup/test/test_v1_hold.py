"""V1 on SimArm: a Ctrl-C on the launch leaves the daemon holding, a relaunch does not jump."""

import os
import signal
import subprocess
import time

from v1 import DAEMON_LOG, goals, max_dq, print_tail, status, wait_for_controllers

TOL = 1e-3  # rad, and m for the gripper


def launch(log):
    # A process group of its own, so a SIGINT reaches every node as a Ctrl-C does. The daemon
    # is not in it: the plugin starts the daemon in a new session. Unbuffered, so the log is
    # whole when a failed check prints it and the SIGKILL comes before launch flushes.
    return subprocess.Popen(["ros2", "launch", "maker_arm_bringup", "simarm.launch.py",
                             "persist:=true"],
                            start_new_session=True, stdout=open(log, "w"),
                            stderr=subprocess.STDOUT, env={**os.environ, "PYTHONUNBUFFERED": "1"})


def ctrl_c(p):
    os.killpg(p.pid, signal.SIGINT)
    p.wait(timeout=120)  # move_group can segfault, servo_node needs SIGKILL after 15 s


def settled_q(timeout=10.0):
    # random_goals waits only until the jaw is within 2 mm of its goal, so the jaw can still
    # be closing when it exits.
    q = status()["q"]
    for _ in range(int(timeout / 0.5)):
        time.sleep(0.5)
        q, prev = status()["q"], q
        if max_dq(q, prev) < 1e-5:
            return q
    raise TimeoutError(f"the arm is still moving after {timeout} s: {q}")


def release():
    return subprocess.run(["maker-arm-cli", "release", "--interface", "sim"], input="RELEASE\n",
                          capture_output=True, text=True, timeout=30)


def test_ctrl_c_leaves_the_daemon_holding_and_a_relaunch_does_not_jump(tmp_path):
    procs = []
    try:
        procs.append(launch(tmp_path / "first.log"))
        wait_for_controllers()
        token, payload = goals(1)
        assert token == "GOALS_OK", payload
        q1 = settled_q()
        print(f"q1 {q1}")
        ctrl_c(procs[-1])

        held = status()
        print(f"after Ctrl-C: mode {held['mode']}, max |q - q1| {max_dq(held['q'], q1):.3g}, "
              f"q {held['q']}")
        assert held["mode"] == "holding"
        assert max_dq(held["q"], q1) < TOL

        procs.append(launch(tmp_path / "second.log"))
        wait_for_controllers()
        back = status()
        print(f"after relaunch: mode {back['mode']}, max |q - q1| {max_dq(back['q'], q1):.3g}, "
              f"q {back['q']}")
        assert max_dq(back["q"], q1) < TOL
        ctrl_c(procs[-1])

        r = release()
        assert r.returncode == 0, r.stdout + r.stderr
    except BaseException:
        for log in [*sorted(tmp_path.glob("*.log")), DAEMON_LOG]:
            print_tail(log)
        raise
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait()
        release()  # nothing to do when the daemon is gone
