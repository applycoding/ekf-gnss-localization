# Deviations

## Initialization time (~9.0–9.1 s vs plan ~8.6–8.7 s)

Straight-segment heading change is tracked as the net integrated gyro (`∫ ω_m dt`) over the B-window segment rather than `∫ |ω_m| dt`. With bias 0.01 rad/s this removes spurious restarts from the old integral-of-absolute test. Measured init time on seed 1: **9.1 s** (matched), **9.0 s** (urban).

## `robot_localization` side-by-side CSV (criterion 19, PLAN optional comparison)

`ros2/ekf_gnss_demo` runs **live** fusion (`ekf_gnss_demo_node` subscribes to `/sim/imu`, `/sim/wheel`, `/sim/gnss/pose`). Gated CI uses `launch_testing` and `check_odom_rmse.py` (odom count, finite values, matched RMSE ≤ 1.0 m). A `robot_localization` `ekf_node` plus truth/ours/RL comparison CSV with the ≥ 90% epoch coverage check from PLAN is **not** in this repo. The README does not claim our filter beats `robot_localization`.
