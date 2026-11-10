# Deviations

## Initialization time (~9.0–9.1 s vs plan ~8.6–8.7 s)

Straight-segment heading change is tracked as the net integrated gyro (`∫ ω_m dt`) over the B-window segment rather than `∫ |ω_m| dt`. With bias 0.01 rad/s this removes spurious restarts from the old integral-of-absolute test. Measured init time on seed 1: **9.1 s** (matched), **9.0 s** (urban).

## ROS 2 Humble side-by-side (criterion 19)

The workflow `.github/workflows/ros2-humble.yml` is a manual-only, non-gating placeholder (`ros2-humble-placeholder`) that builds and tests nothing, and the `ros2/ekf_gnss_demo` package is **not** included in this repository yet. The job exits successfully with a notice; a full `robot_localization` comparison CSV is not produced until that package lands.
