#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail

output=$(timeout 60s ros2 launch ekf_gnss_demo ekf_gnss_demo.launch.py replay_rate_hz:=8000.0 2>&1)
echo "$output" | grep -q 'sim_sensor replay finished'
echo "$output" | grep -q 'ekf_gnss_demo replay finished'
echo "$output" | grep -q 'rmse'
