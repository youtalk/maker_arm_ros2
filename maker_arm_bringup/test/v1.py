"""Helpers of the V1 checks on SimArm. maker-arm-cli must be on PATH.

Import it before any process starts: the environment it sets at import time reaches every one.
"""

import collections
import json
import os
import re
import subprocess
import tempfile
import time

CONTROLLERS = ("joint_state_broadcaster", "arm_controller", "gripper_controller")
ANSI = re.compile(r"\x1b\[[0-9;]*m")  # ros2 control colors its output even into a pipe
# The plugin's default daemon log for the sim interface. Each daemon appends to it.
DAEMON_LOG = os.path.expanduser("~/.ros/log/maker-arm-sim.log")

# The plugin and maker-arm-cli put the sim socket there. CI containers do not set it.
if "XDG_RUNTIME_DIR" not in os.environ:
    os.environ["XDG_RUNTIME_DIR"] = tempfile.mkdtemp(prefix="v1-")
# A private ROS graph: the shell's domain can hold another stack with the same node names
# (/controller_manager, /servo_node). Overwrite, the shell may export its own domain.
os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"


def wait_for_controllers(timeout=120.0):
    deadline = time.monotonic() + timeout
    out = ""
    while time.monotonic() < deadline:
        try:
            out = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True,
                                 text=True, timeout=30).stdout
        except subprocess.TimeoutExpired:
            continue
        rows = (line.split() for line in ANSI.sub("", out).splitlines())
        state = {r[0]: r[-1] for r in rows if r}
        if all(state.get(c) == "active" for c in CONTROLLERS):
            return
        time.sleep(1.0)
    raise TimeoutError(f"the controllers are not active after {timeout} s: {out!r}")


def status():
    out = subprocess.run(["maker-arm-cli", "status", "--interface", "sim"], capture_output=True,
                         text=True, timeout=10, check=True).stdout
    return json.loads(out.split("STATUS ", 1)[1])


def goals(count, seed=0):
    """Run random_goals and return its token and payload. The token line goes to the log.

    On any other token than GOALS_OK, its whole stderr (MoveIt's log) goes there too.
    """
    cmd = ["ros2", "run", "maker_arm_demos", "random_goals",
           "--count", str(count), "--seed", str(seed)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30 + 20 * count)
    line = next((line for line in r.stdout.splitlines() if line.startswith("GOALS_")),
                'GOALS_FAIL {"error": "no token line"}')
    print(line, flush=True)
    token, payload = line.split(" ", 1)
    if token != "GOALS_OK":
        print(f"--- random_goals stderr\n{r.stderr}", flush=True)
    return token, json.loads(payload)


def print_tail(path, n=200):
    """Print the last n lines of a log file, for a failed check."""
    try:
        with open(path, errors="replace") as f:
            text = "".join(collections.deque(f, n))
    except OSError as e:
        text = f"({e})\n"
    print(f"--- {path}, last {n} lines\n{text}", flush=True)


def max_dq(a, b):
    return max(abs(x - y) for x, y in zip(a, b, strict=True))
