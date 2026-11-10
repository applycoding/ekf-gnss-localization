#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
exec 2>&1

sim_out=$(timeout 30s ros2 run ekf_gnss_demo sim_sensor_node --ros-args -p replay_rate_hz:=8000.0 2>&1)
echo "$sim_out" | grep -q 'sim_sensor replay'
ekf_out=$(timeout 30s ros2 run ekf_gnss_demo ekf_gnss_demo_node --ros-args -p replay_rate_hz:=8000.0 2>&1)
echo "$ekf_out" | grep -q 'ekf_gnss_demo replay'
echo "$ekf_out" | grep -q 'ekf_gnss_demo replay finished'
echo "$ekf_out" | grep -qi 'rmse'
