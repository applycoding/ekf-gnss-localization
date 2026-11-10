#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail

ros2 launch ekf_gnss_demo ekf_gnss_demo.launch.py replay_rate_hz:=5000.0 &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

timeout 90s ros2 topic echo /ekf_gnss/odometry --once
timeout 90s ros2 topic echo /ekf_gnss/pose --once
timeout 90s ros2 topic echo /sim/imu --once
