# ekf-gnss-localization

Simulation-only portfolio demo: an 8-state extended Kalman filter fusing wheel odometry, a z-gyro / forward accelerometer, and **simulated** WGS84 GNSS (white noise, Gauss–Markov bias, dropouts, multipath), scored against scripted ground truth with RMSE and NEES/NIS checks. **Not** for real vehicles, RTK, Autoware, or certification.

Author: **Arpan Bom**. License: [Apache-2.0](LICENSE).

## What it is not

Same author portfolio repos, different problems: `cpp-flight-controller` (quad EKF), `ros2-nav2-slam-sim` (SLAM/Nav2), `ros2-vision-tracking-demo` (object tracking), `sim-gimbal-stabilizer` (gimbal attitude).

## Observability (honest)

Heading and gyro bias become observable from GNSS motion while moving; at rest heading is not observable and is initialized from GNSS displacement. Wheel scale needs speed and GNSS. GNSS bias states are only weakly separable from position (cross-covariance is modeled). First-order `Q` and Euler predict are validated at **100 Hz** (`dt ≤ 0.02 s`).

## Eight-state design and ablation

States: position, heading, speed, gyro bias, wheel scale, 2D GNSS bias. Without bias states, the white-noise GNSS model is overconfident under Gauss–Markov error (`urban_white_model` Monte Carlo shows NEES well above the χ² upper bound).

## Build (native)

```bash
cmake --preset default && cmake --build --preset default && ctest --preset default
python3 tools/check_no_heap.py
```

## Web demo

WASM build (emsdk **7.0.0**), Vite base `/ekf-gnss-localization/`:

```bash
source emsdk/emsdk_env.sh
cmake --preset wasm && cmake --build --preset wasm
cp build/wasm/ekf.js build/wasm/ekf.wasm web/public/
cd web && npm ci && npx tsc --noEmit && npx vitest run && npm run build
```

## ROS 2 Humble (optional demo)

Package `ros2/ekf_gnss_demo`: `sim_sensor_node` replays simulated IMU, wheel, and GNSS topics; `ekf_gnss_demo_node` publishes `/ekf_gnss/odometry` and `/ekf_gnss/pose` from the same deterministic C++ filter run (not a live ROS-graph fusion loop). This is simulation-only; it does **not** benchmark against `robot_localization` (see [DEVIATIONS.md](DEVIATIONS.md)).

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ekf_gnss_demo
source install/setup.bash
ros2 launch ekf_gnss_demo ekf_gnss_demo.launch.py
```

## How this was verified

See [docs/verification.md](docs/verification.md) for the full map of PLAN criteria **2–19** to GoogleTest names (`Exact2State`, `FilterFrozen`, `Jacobian`, `Geodesy`, `InvalidInputs`, `Gates`, `Scenario`, `MonteCarlo`/`mc`, `Reacquisition`, `Determinism`, `NoHeapOperator`), plus `web.yml` (full WASM parity script, Vitest + Playwright), `core.yml` (`EKF_REQUIRE_WASM_PARITY`, `tools/o0_o2_parity.sh`, `tools/check_no_heap.py`), and `ros2-humble.yml` (`ekf_gnss_demo` launch test). `pages.yml` deploys only after **both** `core` and `web` succeed on the same commit. NEES Monte Carlo bounds are engineering regression checks, not formal χ² tests. Gaps: [DEVIATIONS.md](DEVIATIONS.md) (init timing, no `robot_localization` CSV).

## Idea credits

Conceptual references (not copied): `robot_localization`, PythonRobotics, TinyEKF, GeographicLib/pymap3d, filterpy — see [REFERENCES.md](REFERENCES.md).
