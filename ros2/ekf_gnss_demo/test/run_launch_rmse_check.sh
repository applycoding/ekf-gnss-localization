#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/../../../install/setup.bash"

ros2 launch ekf_gnss_demo ekf_gnss_demo.launch.py \
  scenario:=matched seed:=1 replay_rate_hz:=500.0 &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 5
/usr/bin/python3 "${SCRIPT_DIR}/check_odom_rmse.py" \
  --min-messages 4000 \
  --rmse-max-m 1.0 \
  --timeout-s 90.0
