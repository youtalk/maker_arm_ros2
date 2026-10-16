#!/usr/bin/env bash
# Bring up vcan0, the virtual CAN bus that the MuJoCo plant and the daemon share.
# Run it once per boot, as root: sudo bash vcan_up.sh
set -euo pipefail
modprobe vcan
ip link show vcan0 > /dev/null 2>&1 || ip link add dev vcan0 type vcan
ip link set up vcan0
