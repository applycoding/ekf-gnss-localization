# Deviations

## Initialization time (~9.0–9.1 s vs plan ~8.6–8.7 s)

Straight-segment heading change is tracked as the net integrated gyro (`∫ ω_m dt`) over the B-window segment rather than `∫ |ω_m| dt`. With bias 0.01 rad/s this removes spurious restarts from the old integral-of-absolute test. Measured init time on seed 1: **9.1 s** (matched), **9.0 s** (urban).

## `robot_localization` side-by-side CSV (criterion 19, PLAN illustrative comparison)

The `ros2/ekf_gnss_demo` package replays the C++ simulator and filter trace on ROS topics (`sim_sensor_node` + `ekf_gnss_demo_node`) and is gated in `ros2-humble.yml`. A live topic-fed fusion node and a `robot_localization` `ekf_node` recording truth/ours/RL in one CSV is **not** wired in v1: the demo nodes replay precomputed `RunSimulation` / `RunScenario` output for deterministic CI rather than closing the ROS graph around two independent estimators. The README does not claim our filter beats `robot_localization`; see PLAN for why any such plot would be illustrative only.
