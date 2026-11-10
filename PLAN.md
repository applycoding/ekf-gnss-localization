# ekf-gnss-localization — v1 plan

Date 2026-11-10. Planning is complete; this file is the build contract. Outbound license **Apache-2.0**.

## Goal

A public portfolio demo for Arpan Bom that shows planar vehicle localization, GNSS error handling, sensor fusion and filter consistency. It is aimed at the localization and GNSS gap for a TIER IV autonomous-driving software interview.

The artifact is a **simulated** 2D ground vehicle. An extended Kalman filter (EKF) fuses wheel speed, an IMU (z-gyro and forward accelerometer) and GNSS position fixes. The GNSS simulator emits WGS84 latitude, longitude and height with white noise, a slowly varying correlated error (first-order Gauss–Markov), a 25 s dropout and multipath outliers. The demo plots the estimate against ground truth, position error with a covariance envelope, RMSE, and NEES / NIS against chi-square bounds, and it compares four baselines.

This is a **portfolio demo, simulation only**. Do not claim Autoware, radar, CAN, HILS, RTK, real receivers, real vehicles, production localization or safety certification.

Git author and committer for every commit: **Arpan Bom <applycoding@gmail.com>**. No `Co-authored-by` trailers, and no AI mention in commits, README, code or docs.

## Scope

- `ekf_core` (the interview artifact): WGS84 geodetic ↔ ECEF ↔ ENU, angle wrap, fixed-size matrix helpers, the 8-state EKF (predict, wheel update, GNSS update, chi-square gates, gate-lockout recovery) and the initializer state machine. No heap, no ROS, no JSON, no `<vector>`.
- `ekf_sim` (host, tests and WASM only): scripted truth trajectory, sensor and GNSS error models with a portable RNG, an integer-tick scheduler, baselines, and metrics (RMSE, NEES, NIS, rejection counts).
- Tests: GoogleTest. Core ctest needs no Python, Node or ROS.
- Browser: the same C++ core and sim compiled to WASM. A TypeScript + Vite UI with Canvas 2D plots. There is no second filter in TypeScript.
- Optional ROS 2 Humble package with a `robot_localization` side-by-side comparison. It never gates core CI.
- Optional pybind11 bindings for offline Monte Carlo plots. Off by default.

**Not a duplicate of other applycoding repos** (licenses checked 2026-11-10 with `gh repo list applycoding`):

| Repo | What it is | How this repo differs |
| --- | --- | --- |
| `cpp-flight-controller` (MIT) | Quadrotor attitude control, complementary filter, error-state EKF, altitude KF | Planar position, heading, gyro bias, wheel scale and GNSS bias. No quaternion attitude, no quad, no altitude loop. |
| `ros2-nav2-slam-sim` (MIT) | Gazebo + slam_toolbox + Nav2 | No LiDAR, no SLAM, no Gazebo. GNSS/odometry/IMU fusion only. |
| `ros2-vision-tracking-demo` (Apache-2.0) | Kalman tracker over YOLO detections | Different problem (object tracking, not ego localization). |
| `sim-gimbal-stabilizer` (Apache-2.0) | Gimbal attitude ESKF and PID | Different problem (attitude control, not GNSS localization). |

Everything is written from first principles. Nothing is copied or translated from any project.

## Architecture

**Decision:** two static libraries. `ekf_core` holds the filter and geodesy. `ekf_sim` holds truth, sensors, scheduler, baselines and metrics, and links into tests, WASM, the optional ROS node and the optional pybind module. The browser loads one WASM module.

**Why:** the interview topic is the filter and the GNSS error model, so the WASM demo must run the same C++ filter. Keeping trace buffers in `ekf_sim` keeps the core heap-free.

### Frames

- Local ENU tangent plane in meters. Origin `lat0 = 35.6812°`, `lon0 = 139.7671°`, `h0 = 40 m`. This is a fixed Tokyo tangent point, not a claim of a mapped drive.
- Heading `ψ` is measured counter-clockwise from East (+x). Body frame: x forward, y left, z up (REP-103).
- GNSS generation order: true ENU position + GNSS errors (in ENU) → ECEF → LLH. The filter converts LLH → ECEF → ENU with the same origin before each update. The vertical channel is ignored (planar filter). Antenna lever arm is exactly zero.
- WGS84: `a = 6378137 m`, `f = 1/298.257223563`, `e² = f(2 − f)`, `b = a(1 − f) = 6356752.314245179 m`.
- Forward conversion is closed form. **Inverse conversion is iterative** (latitude fixed-point on `tan φ = z / (p (1 − e² N/(N+h)))`, at most 10 iterations, stop when `|Δφ| < 1e-14 rad`). A one-step Bowring inverse was measured on the box at up to `1.03e-6 m` round-trip error over the test domain, which fails the `1e-6 m` criterion, so it is not used. ECEF `(0,0,0)` returns `InvalidInput`.

### State (locked)

`x = [p_e, p_n, ψ, v, b_g, k, β_e, β_n]` (indices 0–7).

| Index | State | Unit | Meaning |
| --- | --- | --- | --- |
| 0, 1 | `p_e, p_n` | m | Vehicle position in ENU |
| 2 | `ψ` | rad | Heading, wrapped to `(−π, π]` |
| 3 | `v` | m/s | Signed forward speed |
| 4 | `b_g` | rad/s | z-gyro bias |
| 5 | `k` | – | Wheel-speed scale factor (`z_wheel = k·v`) |
| 6, 7 | `β_e, β_n` | m | GNSS correlated (Gauss–Markov) error |

Two configuration flags, set per scenario and recorded in every trace:

- `model_gnss_bias` (default true). When false, `β` rows and columns of `P` are zero, `β` stays 0, and the GNSS model is white with `R = σ_rep² I`.
- `estimate_wheel_scale` (default true). When false, `k` is fixed at 1 with zero variance.

**Why 8 states and not 5:** verified on the box (see Verification). A 5-state filter that treats the 60 s GNSS error as white, even with `R` inflated to `σ_rep = 2.5 m`, has a Monte Carlo position NEES around 21–27 against an upper bound of 2.59. That is badly overconfident. Augmenting `β` brings the urban position NEES back inside the bounds. The 5-state behaviour is kept as an **ablation** (criterion 14) because it is a useful lesson.

Observability, stated in the README: heading and gyro bias become observable only from GNSS displacement while moving. At rest, absolute heading is not observable. `k` needs non-zero speed and GNSS. `β` and position are only separated through the motion model, so their split is weakly observable; the cross-covariance handles this honestly.

### Prediction (locked)

Forward Euler on the IMU tick. Inputs `ω_m` (gyro z) and `a_m` (forward accel) are held over the step.

- `p_e += v cos ψ · dt`, `p_n += v sin ψ · dt`
- `ψ = wrap(ψ + (ω_m − b_g) dt)`, `v += a_m dt`
- `b_g`, `k` unchanged. `β ← φ β` with `φ = exp(−dt/τ)`, `τ = 60 s`.

Jacobian `F` = identity plus `F[0,2] = −v sin ψ dt`, `F[0,3] = cos ψ dt`, `F[1,2] = v cos ψ dt`, `F[1,3] = sin ψ dt`, `F[2,4] = −dt`, `F[6,6] = F[7,7] = φ`.

Process noise (first-order discretization; a documented approximation that the Monte Carlo NEES validates at `dt = 0.01 s`):
`Q = diag(0, 0, q_ω dt, q_a dt, q_b dt, q_k dt, σ_β²(1 − φ²), σ_β²(1 − φ²))` with

- `q_ω = (0.002 rad/√s)²` (gyro white noise PSD, `4e-6 rad²/s`)
- `q_a = (0.05 m/s/√s)²` (accel white noise PSD, `2.5e-3 m²/s³`)
- `q_b = (1e-4 rad/s/√s)²` (bias walk; the truth bias is constant, so this is a small, labelled inflation)
- `q_k = (1e-5 /√s)²`
- `σ_β = 1.0 m` (only when `model_gnss_bias`)

The public `Predict` accepts `0 < dt ≤ 0.02 s` only. Larger steps return `InvalidDt`, because the first-order `Q` and Euler step are validated only at the 100 Hz rate.

The truth trajectory is integrated with a **different** integrator (midpoint, 100 Hz) from the filter, so the filter is not given its own model as truth.

### Updates (locked)

- Joseph form `P ← (I − KH) P (I − KH)ᵀ + K R Kᵀ`, then `P ← ½(P + Pᵀ)`.
- Innovation covariance `S` is solved with a fixed-size Cholesky (1×1 or 2×2). A failed factorization returns `Numerical` and rolls back. Never form a general 8×8 inverse.
- After every predict and update, check that every entry of `x` and `P` is finite and that the active diagonal is `> 0`. On failure return `Numerical`, restore the previous `x` and `P`.
- **Wheel:** `z = k v + n`, `H[3] = k`, `H[5] = v` (`H[5] = 0` if scale is not estimated), `R = (0.05 m/s)²`.
- **GNSS:** `z = p + β + n` with `H = [I₂ 0 0 0 0 I₂]` and `R = (1.5 m)² I` when `model_gnss_bias`; otherwise `H = [I₂ 0…]`, `R = σ_rep² I`.
- GNSS has no angular innovation. Heading wrap applies to the state and to NEES heading error.

### Gates and recovery (locked)

- GNSS gate: reject when NIS `> 9.210340371976184` (chi-square, 2 DOF, 99%; equals `−2 ln 0.01`).
- Wheel gate: reject when NIS `> 6.634896601021214` (chi-square, 1 DOF, 99%).
- Gate off (`gate = false`) disables **both** gates. This is the "EKF, gate off" baseline.
- Every candidate's **pre-gate** NIS is logged with an accepted/rejected flag. Accepted-only NIS is never reported as a consistency statistic (it is truncated by construction).
- **GNSS reacquisition:** after 20 consecutive gated fixes (2 s), check that their innovations are mutually consistent: every innovation is within `3·√2·σ_meas` of the component-wise median of the 20. If consistent, add `NIS_last · R` to the 2×2 position block of `P`, apply the current fix, increment `reacquisitions` and set status `Reacquired`. If not consistent, clear the window and keep rejecting. Outlier bursts point in random directions and fail the check; a diverged filter sees consistent innovations and passes it.
- **Wheel lockout:** after 100 consecutive gated wheel samples (2 s), add `y²` to `P[3,3]` and apply the sample. The scripted slip windows are 1 s, so they never trigger this. (A lockout window equal to the slip length was tried on the box and broke NEES consistency; 2 s fixed it.)
- No robust or soft-reweighting mode in v1.

### Initialization (locked state machine)

States: `NoFix → Collecting → Initialized`. Also `Reacquired` as a transient status.

1. `NoFix`: wait for valid GNSS. `Predict` returns `NotInitialized` and changes nothing. The initializer accumulates odometry distance `s = ∫ z_wheel dt` and integrated gyro heading change.
2. `Collecting`: window A is the first 5 valid fixes. Continue until the odometry distance since the last fix of A is at least `20 m`, then window B is the last 5 fixes. The segment must be straight: integrated `|ω_m|` change over the window `< 0.1 rad`. If not, restart A from the current fix. Wheel speed must be positive (forward).
3. Initialize:
   - `ψ₀ = atan2(Δn, Δe)` of `median(B) − median(A)` (component-wise medians, robust to 1–2 outliers per window)
   - `p₀ = median over j in B of (fix_j + (s(t₀) − s(t_j)) · [cos ψ₀, sin ψ₀])`
   - `v₀` = latest wheel speed, `b_g = 0`, `k = 1`, `β = 0`
   - `P₀ = diag(σ_p², σ_p², σ_ψ², 0.5², 0.02², 0.03², σ_β², σ_β²)`
   - `σ_p² = 1.5² + σ_β²` (bias model on) or `σ_rep²` (off)
   - `σ_ψ = clamp(3 · σ_meas / |Δ|, 0.05, 0.5) rad`
   - With the bias model on, `P[p_e, β_e] = P[p_n, β_n] = −σ_β²`, because a fix measures `p + β`. On the box this cross-covariance raised the urban epoch-in-bounds fraction from 0.82–0.89 to 0.94–1.0.
4. In both scripted scenarios, initialization completes at `t ≈ 8.6–8.7 s` on all 200 seeds tested. This is reported, not assumed: a run that never meets the gate stays `Collecting` and reports it.

### Time and scheduling

The sim uses integer ticks: tick `i` is `t = i · 0.01 s`. IMU on every tick, wheel when `i % 2 == 0` (50 Hz), GNSS when `i % 10 == 0` (10 Hz). Fixed order per tick: predict, then wheel, then GNSS. The core never reads a wall clock. A measurement whose `t_s` differs from the filter time by more than `1e-9 s` returns `TimeMismatch` (past or future). There is no retrodiction or reorder buffer; the ROS adapter keeps its own bounded queue.

### Sensor and GNSS models (locked numbers)

Independent named RNG streams per source (gyro, accel, wheel, GNSS white, GNSS Gauss–Markov, outlier draw, outlier vector). Turning the gate on or off, or changing the dropout, never changes the other streams' draws.

- **RNG:** `ekf_sim` implements xoshiro256** seeded by splitmix64, and its own Gaussian transform (Box–Muller). `std::normal_distribution` and `std::uniform_real_distribution` are banned because their output is implementation-defined and differs between libstdc++ and Emscripten's libc++.
- Gyro: `ω_m = ω + b + n`, per-sample `σ = 0.02 rad/s` (= `0.002/√0.01`), constant `b = 0.01 rad/s` unless a test overrides it.
- Accel: `a_m = dv/dt + n`, per-sample `σ = 0.5 m/s²` (= `0.05/√0.01`). No accel bias. Lateral/centripetal acceleration and gravity are not modelled (planar, no sideslip). Say so.
- Wheel: `z = s_true · v + n`, `σ = 0.05 m/s`. Longitudinal slip only: during a slip window `z` is multiplied by `0.5`. No lateral sideslip.
- GNSS white: `σ = 1.5 m` per horizontal axis.
- GNSS Gauss–Markov: `β ← φβ + σ_β √(1 − φ²) w`, `σ_β = 1.0 m`, `τ = 60 s`, evolved every tick (including during dropouts), `β₀ ~ N(0, σ_β²)`.
- Multipath: each fix is an outlier with probability `0.05`; offset length uniform in `[10, 30] m` at a uniform angle. The filter never sees the outlier label; labels are used only for scoring.

### Scenarios (locked)

| Scenario | GNSS | Wheel | Filter config |
| --- | --- | --- | --- |
| `matched` (default) | White only, no dropout, no outliers | `s_true = 1`, no slip | `model_gnss_bias = false`, `σ_rep = 1.5 m`, scale estimated, gates on |
| `urban` | White + Gauss–Markov + dropout `[50, 75) s` + 5% multipath | `s_true = 1.02`, slip `[40, 41) s` and `[90, 91) s` | `model_gnss_bias = true`, scale estimated, gates on |
| `urban_white_model` (ablation) | Same as urban | Same as urban | `model_gnss_bias = false`, `σ_rep = 2.5 m` |

### Trajectory (locked, exact)

Speed `v(t)` and yaw rate `ω(t)` are piecewise, starting at the origin heading East, duration `125 s`:

- `v`: `8·t/15` for `t < 15`; `8` for `15 ≤ t < 115`; `8 − 0.8 (t − 115)` for `115 ≤ t < 125`; `0` at `t = 125`.
- `ω`: `8/25 = 0.32 rad/s` for `40 ≤ t < 70` (left turn, radius 25 m, 9.6 rad ≈ 1.53 laps); `+0.4 rad/s` for `85 ≤ t < 100` and `−0.4 rad/s` for `100 ≤ t < 115` (an S-reversal on 20 m radius arcs, 6 rad each, not two full circles); `0` otherwise.
- `dv/dt` for the accelerometer is the analytic derivative of `v(t)`.

The dropout `[50, 75) s` falls inside the left turn, the hardest place for dead reckoning.

### Metrics and baselines

Per run (after initialization): horizontal RMSE; max horizontal error; RMSE and max during the dropout; error 3.0 s after the first returning fix; pre-gate NIS for all candidates; gated counts; outlier recall and false-rejection rate (scored with simulator labels); reacquisitions; final `b_g`, `k`.

- **Position NEES** = `e_pᵀ P_pp⁻¹ e_p` with the **marginal** top-left 2×2 block. Not a Schur complement.
- **Full NEES** uses the 5 core states `[p_e, p_n, ψ, v, b_g]` and their marginal 5×5 block, with the heading error wrapped.
- Both are posterior values at the GNSS epoch (10 Hz), after that tick's updates.
- **Monte Carlo:** `N = 50` seeds (1–50). Epochs run from `max(t_init) + 5 s` to `125 s` at 10 Hz. The epoch-wise mean NEES over the 50 runs is compared with two-sided 95% bounds `χ²_{0.025}(N n)/N` and `χ²_{0.975}(N n)/N` (SciPy `chi2.ppf`, constants frozen in the test):
  - position, df 100: **[1.484439, 2.591224]**
  - full, df 250: **[4.161956, 5.913773]**
- The time-mean and the epoch fraction are engineering regression checks, not a formal 95% test (epochs within a run are correlated). The README says so, and reports upper and lower violations separately.
- **Error envelope in plots:** radial `sqrt(11.829007 · λ_max(P_pp))` (`11.829007 = χ²_{0.9973}(2)`), labelled as a 99.73% bound on the largest axis. Component errors with their own ±3σ bands are also shown.
- **Single-run NEES reference (browser):** a single seed is compared with single-run 95% intervals, position `[0.050636, 7.377759]` and full `[0.831212, 12.832502]`, never with the N = 50 bounds.

Baselines on the same truth and the same sensor draws:

| Baseline | Definition |
| --- | --- |
| (a) Dead reckoning | From the true pose at the init time, integrate wheel speed along a heading integrated from the raw gyro (no bias correction, no GNSS) |
| (b) GNSS-only | Raw ENU fixes at fix epochs only, no interpolation |
| (c) EKF, gate off | Same filter, both gates disabled |
| (d) EKF, gate on | Default |

### Matrix library

Hand-written fixed-size `Mat<R,C>` / `Vec<N>` (`double`, row-major, `N ≤ 8`), no heap, no Eigen. Only 1×1 and 2×2 Cholesky solves are needed for updates, and a 5×5 Cholesky solve for NEES (in `ekf_sim`).

### ROS 2 Humble (optional, never gates core)

Package `ros2/ekf_gnss_demo`.

- A sim node publishes `sensor_msgs/Imu` (angular velocity z and linear acceleration x only, orientation covariance `−1`), wheel `nav_msgs/Odometry` (twist `vx` only), and GNSS as `sensor_msgs/NavSatFix`. It also publishes the same fixes already converted by our fixed-origin adapter to ENU as a `nav_msgs/Odometry` pose in frame `map` with `σ = 1.5 m` covariance.
- Our node publishes `nav_msgs/Odometry` on `/ekf_gnss/odometry` in `map`, child `base_link`.
- **Fair comparison:** `robot_localization` `ekf_node` gets the **same information**: `two_d_mode: true`, `frequency: 100`, wheel `vx` only, IMU `vyaw` and `ax` only, converted GNSS pose `x, y` only, differential false, `world_frame: map`, `publish_tf: false`, `pose0_rejection_threshold: 3.0349` (the Mahalanobis distance equal to `√9.2103`). `navsat_transform_node` is **not** used in v1, because it needs an earth-referenced heading that this IMU does not provide.
- Static `base_link → imu_link` and `base_link → gps_link` transforms (zero offset). Neither estimator broadcasts `map → base_link`. Use `/clock` with `use_sim_time`, and pace the sim at real time.
- The launch file records a CSV of truth, our estimate and `robot_localization`'s estimate. The README compares the plots and RMSE in words and does not claim a winner. Different state models and gating mean the comparison is illustrative.

### Browser

- emsdk **7.0.0**, never `latest`. Single-threaded WASM (no pthreads; Pages has no cross-origin isolation headers). A small C ABI or Embind boundary that returns typed-array traces.
- Vite `base` `/ekf-gnss-localization/`. The `.wasm` URL resolves through `import.meta.env.BASE_URL` / `locateFile`. The Run button is enabled only after the module factory resolves; load errors are shown.
- Canvas 2D: trajectory map (truth, EKF, raw fixes, rejected fixes in a different marker), error vs time with the envelope, NEES vs single-run bounds, and an RMSE table for all four baselines.
- Controls: scenario preset (matched / urban / urban white model), GNSS white sigma, outlier probability, dropout length, gate on/off, model GNSS bias on/off, seed, Run. When the filter model no longer matches the simulator, the page labels NEES "diagnostic: model mismatch".

### Python (optional)

pybind11 module plus `python/bench/monte_carlo.py` for the README figures. A native CSV runner (`ekf_cli`) is enough for core CI; Python is never required for criteria 1–17.

## Pinned stack

Checked 2026-11-10 (box `gh api` / `npm view` / PyPI / nodejs.org plus an independent review). Pins are exact. FetchContent uses `GIT_TAG <full SHA>`, or `URL` + `URL_HASH` decided before the first build. Record actual compiler, CMake, Node, npm and apt versions in CI logs.

| Component | Pin |
| --- | --- |
| Language | C++17, `CMAKE_CXX_EXTENSIONS OFF`, `-ffp-contract=off`, no `-ffast-math`, no Eigen |
| Core CI | `ubuntu-24.04`, `g++-13` selected explicitly, CMake ≥ 3.22 (log actual) |
| GoogleTest | v1.18.0, commit `063de7e9578f82b369302001269680b4b1553359` (BSD-3-Clause) |
| pybind11 (optional) | v3.1.0, commit `97bf890db679505a14dfe547a5e77bb2bd05dc90` (BSD-3-Clause) |
| nlohmann/json | v3.12.0, commit `55f93686c01528224f448c19128836e7df245f72` (MIT). Scenario files in `ekf_sim` / CLI only, never in core headers. |
| Python (optional) | 3.12; pytest 9.2.0; numpy/scipy/matplotlib exact pins with hashes in `python/requirements-dev.txt` if used |
| Node | 26.13.0 (Active LTS "Lithium"); npm version and lockfile committed; `npm ci` |
| TypeScript / Vite / Vitest | 7.0.2 / 8.3.7 / 5.1.0 |
| Playwright | `@playwright/test` 1.65.0, Chromium only, `npx playwright install --with-deps chromium` |
| emsdk | 7.0.0, commit `1693ce7359fc9694f198694f902a2cc061239b86`; `./emsdk install 7.0.0 && ./emsdk activate 7.0.0`; log `emcc --version`. Re-assert Node 26 on PATH after activation. |
| Humble CI | `ros:humble-ros-base-jammy`, digest + `linux/amd64` resolved and pinned when the workflow is first written |
| ROS apt | `ros-humble-robot-localization` (rosdistro Humble release 3.5.5-1; public main repo may still be 3.5.4-1; record `apt-cache policy`), `ros-humble-rclcpp`, `ros-humble-nav-msgs`, `ros-humble-sensor-msgs`, `ros-humble-geometry-msgs`, `ros-humble-tf2-ros`, `ros-humble-ament-cmake` |
| Actions | Full commit SHAs with a version comment, listed in `.github/workflow-pins.md` |
| Plots | Canvas 2D. No chart library. |

Humble is supported until May 2027; the README notes that.

## File layout

```text
LICENSE  NOTICE  README.md  CMakeLists.txt  CMakePresets.json  .gitignore
cpp/include/ekf/{types.hpp, mat.hpp, geodesy.hpp, filter.hpp, initializer.hpp}
cpp/src/core/{geodesy.cpp, filter.cpp, initializer.cpp}
cpp/include/ekf_sim/ + cpp/src/sim/{rng, trajectory, sensors, scenario, scheduler, baselines, metrics}.cpp
cpp/tools/ekf_cli.cpp            (runs a scenario, writes CSV/JSON)
cpp/test/                         (GoogleTest; see Test plan)
tools/check_no_heap.py            (bans <vector>/<string>/<memory>/new/malloc in core sources)
scenarios/{matched,urban,urban_white_model}.json
web/                              (Vite + TS, src/, wasm glue, tests/, playwright.config.ts)
python/ (optional)                ros2/ekf_gnss_demo/ (optional)
.github/workflows/{core,web,pages,ros2-humble}.yml  .github/workflow-pins.md
docs/demo.png  docs/figures/ (Monte Carlo plots)  docs/verification.md
```

Options: `EKF_BUILD_TESTS` ON; `EKF_BUILD_PYTHON`, `EKF_BUILD_ROS`, `EKF_BUILD_WASM` OFF. Optional dependencies are only searched for and fetched when their option is ON. Namespace `ekf` (core) and `ekf::sim`.

## Interfaces

```cpp
namespace ekf {
enum class Status : uint8_t { Ok, Gated, Reacquired, NotInitialized, InvalidDt,
                              InvalidInput, TimeMismatch, Numerical };
enum class Lifecycle : uint8_t { NoFix, Collecting, Initialized };

struct Llh { double lat_rad, lon_rad, h_m; };
struct Enu { double e, n, u; };
struct Origin { Llh llh; double ecef[3]; double R_enu_from_ecef[9]; };  // built once

struct ImuSample   { double t_s, gyro_z_rad_s, accel_x_m_s2; };
struct WheelSample { double t_s, speed_m_s; };
struct GnssSample  { double t_s, lat_rad, lon_rad, h_m; };

struct Config {                      // validated by ValidateConfig(); invalid -> InvalidInput
  double q_gyro, q_accel, q_bias, q_scale;    // PSDs as in the plan
  double r_wheel;                              // (m/s)^2
  double gnss_sigma_m;                         // white sigma used in R
  double gnss_bias_sigma_m, gnss_bias_tau_s;   // used when model_gnss_bias
  bool model_gnss_bias, estimate_wheel_scale, gate;
};

struct Filter {                      // POD, caller-owned
  double x[8]; double P[64]; double t_s;
  Lifecycle lifecycle;
  uint32_t gnss_accepted, gnss_gated, wheel_accepted, wheel_gated, reacquisitions;
  uint32_t gnss_consec_gated, wheel_consec_gated;
  double gated_innov[20][2]; uint8_t gated_count;   // reacquisition window
  InitializerState init;             // fixed arrays for windows A/B, odometry distance, heading change
};

struct UpdateResult { Status status; double nis_pre_gate; bool accepted; };  // nis = NaN if none

Status Reset(Filter&, const Config&, double t0_s);
Status Predict(Filter&, const Config&, const ImuSample&, double dt_s);
UpdateResult UpdateWheel(Filter&, const Config&, const WheelSample&);
UpdateResult UpdateGnss(Filter&, const Config&, const Origin&, const GnssSample&);
double WrapPi(double);                                // result in (-pi, pi]
Status MakeOrigin(const Llh&, Origin&);
void GeodeticToEcef(const Llh&, double ecef[3]);
Status EcefToGeodetic(const double ecef[3], Llh&);    // iterative; (0,0,0) -> InvalidInput
void EcefToEnu(const Origin&, const double ecef[3], Enu&);
void EnuToEcef(const Origin&, const Enu&, double ecef[3]);
}
```

Every call that returns anything other than `Ok`, `Gated` or `Reacquired` leaves `Filter` bit-identical to its input (the counters too). Truth-dependent metrics (NEES, RMSE) live only in `ekf::sim`.

## Edge cases

- `dt ≤ 0`, `dt > 0.02`, NaN or Inf anywhere → `InvalidDt` / `InvalidInput`, no change.
- Measurement time mismatch → `TimeMismatch`, no change.
- `Predict` before `Initialized` → `NotInitialized`; the initializer still accumulates odometry and gyro.
- Startup inside a dropout: stays `NoFix` until fixes arrive. No output pose is published and metrics exclude it.
- Dropout: no GNSS updates; `P` grows; the next in-gate fix corrects. The Gauss–Markov truth keeps evolving.
- Stop at `t = 125 s` and low speed: heading becomes less certain but the state is not reset. Wheel speed near zero is a valid measurement. Reverse driving is out of the scripted trajectory; initialization refuses non-positive wheel speed (documented limit).
- Slip: wheel gate rejects the samples; the lockout rule covers longer slips.
- Pole / antimeridian in geodesy tests: longitude is not compared at the poles; longitudes are compared modulo `2π`.
- Counters use `uint32_t`; a 125 s run is far from overflow.

## Test plan

**Unit (GoogleTest):**

- `WrapPi` at `±π`, `±3π`, `−0.0`, and NaN → NaN.
- 2-state exact example (criterion 2): `x = [0, 1]`, `P = I`, `F = [[1,1],[0,1]]`, `Q = 0` → `x⁻ = [1, 1]`, `P⁻ = [[2,1],[1,1]]`. Update `z = 2`, `H = [1 0]`, `R = 1` → `S = 3`, `K = [2/3, 1/3]`, `x⁺ = [5/3, 4/3]`, Joseph `P⁺ = [[2/3,1/3],[1/3,2/3]]`, `NIS = 1/3`. Tolerance `1e-12`.
- 8-state frozen example (criterion 2), bias model on, `dt = 0.02`: `x = [10, −5, 0.5, 8, 0.01, 1, 0.3, −0.2]`, `P = diag(2, 2, 0.04, 0.25, 4e-4, 9e-4, 1, 1)` with `P[0,6] = P[1,7] = −1`, gyro `0.31`, accel `0.2`, PSDs as above. Expected values (tolerance `1e-9`):
  - after predict: `x = [10.140413209900, −4.923291913823, 0.506, 8.004, 0.01, 1, 0.299900016665, −0.199933344443]`, `P` diagonal `[2.000312380335, 2.000811619665, 0.04000024, 0.25005, 4.000002e-4, 9.00000002e-4, 1, 1]`
  - after wheel `z = 8.1`: `v = 8.081383013426`, `k = 1.002229299247`, `NIS = 0.029709135329`
  - after GNSS `z_ENU = (12.0, −3.5)`: `p = (10.621255435500, −4.423126700443)`, `ψ = 0.507332004748`, `NIS = 1.556809790907`, `P[0,0] = 1.692285622328`, and `P` stays symmetric positive definite (min eigenvalue `> 0`)
- Jacobian vs central differences (`h = 1e-6`, yaw differences unwrapped): `|F − F_fd| ≤ 1e-7 + 1e-6·|F|` element-wise, on 50 random states including `v = 0`, `v < 0` and `ψ` near `±π`. (Box check: max difference `1.4e-10`.)
- Geodesy: anchors `(0°,0°,0) → (6378137, 0, 0)`; `(0°,90°,0) → (0, 6378137, 0)`; poles → `(0, 0, ±6356752.314245179)`; Tokyo `(35.6812°, 139.7671°, 40 m) → (−3959690.802569, 3350097.500459, 3699540.124670)`; `(45°, 45°, 1000 m) → (3194919.145061, 3194919.145061, 4488055.515647)`. All within `1e-4 m`; the box cross-checked them against pymap3d 3.x within `1e-9 m`. ENU basis at `(0°,0°)`: east = ECEF +Y, north = +Z, up = +X. 1000 seeded round-trips (lat in `[−89°, 89°]`, h in `[−500, 9000] m`) within `1e-6 m` ECEF and `1e-12 rad` latitude.
- Gates: accept NIS just below `9.210340371976184` and reject just above; same for `6.634896601021214`.
- Invalid inputs and forced numerical failure (e.g. a non-PD `P`) leave `Filter` bit-identical.
- Initializer (deterministic): noise-free fixes plus one injected 25 m outlier in window A and one in window B give `|ψ₀ − ψ_true| < 0.02 rad`; a window with a 0.2 rad heading change restarts; non-positive wheel speed never initializes. (Box, random seeds: initial heading error max 0.19 rad, p95 0.12 rad over 200 seeds, consistent with `σ_ψ ≈ 0.2 rad`.)
- No-heap: `tools/check_no_heap.py` passes, and a test binary with replaced global `operator new` / `new[]` / aligned / nothrow variants (all abort) runs Reset → Predict → both updates → invalid paths.

**Scenario tests (seeds 1–5 each; thresholds hold for every seed):** criteria 6–11 and 15.

**Monte Carlo tests (seeds 1–50):** criteria 12–14 and 16. They run in the core workflow as a separate ctest label `mc` (they finish in seconds in C++).

**Determinism:** same seed → bit-identical traces in one process and between `-O0` and `-O2` builds within `1e-9 m`. WASM vs native, full matched run: position within `1e-6 m`, identical gate decisions, RMSE within `1e-6 m`.

**Web (Vitest + Playwright):** the production build is served under `/ekf-gnss-localization/` with `vite preview`. The test asserts the page URL path, waits for a `data-state="done"` attribute, checks that the RMSE text is a finite number, that the trace length is the expected sample count, that each canvas has more than 2% non-background pixels, and that there are no console errors, page errors or failed requests. It then writes `docs/demo.png` at the repo root (a single worker).

**Humble (optional workflow):** launch the comparison for the urban scenario; the CSV has finite, non-constant columns for both estimators covering at least 90% of the expected epochs. The job's own status is visible (not `continue-on-error`) but it is not a required check.

## Numbered success criteria

1. `cmake --preset default && cmake --build --preset default && ctest --preset default` passes on `ubuntu-24.04` with `g++-13`, with Python, Node and ROS options OFF and no optional dependency fetched.
2. The 2-state exact example matches within `1e-12`, and the 8-state frozen example matches every listed value within `1e-9`.
3. The Jacobian finite-difference test passes on all 50 states with the mixed tolerance above.
4. All geodesy anchors and vectors match within `1e-4 m`; the 1000 round-trips meet `1e-6 m` and `1e-12 rad`; the ENU basis test passes.
5. Invalid, time-mismatched and numerically failing inputs return the documented status and leave the filter bit-identical.
6. Matched scenario, seeds 1–5: horizontal RMSE `≤ 1.0 m` **and** `≤ 0.5 ×` raw GNSS RMSE on the same run. (Box: max `0.48 m` over 200 seeds; raw GNSS `2.07–2.19 m`.)
7. Urban scenario, seeds 1–5: gated RMSE `≤ 4.0 m` **and** `≤ 0.5 ×` the gate-off RMSE. (Box: gated max `2.92 m`; gate-off min `16.2 m`.)
8. Urban dropout, seeds 1–5: max error during `[50, 75) s` `≤ 8 m`, and error 3.0 s after the first returning fix `≤ 5 m`. (Box max: `4.62 m` and `3.70 m`.)
9. Dead reckoning (baseline a) error at `t = 90 s` is `> 20 m` while the EKF's is `≤ 4 m`, in both scenarios, seeds 1–5. (Box: DR min `55 m`; EKF max `2.1 m`.)
10. Matched scenario: `|b̂_g − b| < 0.004 rad/s` at `t = 40 s` for `b ∈ {−0.01, 0, +0.02}`, seeds 1–5. A mutant with bias estimation disabled fails this check for `b = 0.01`. (Box max error `0.0013`.)
11. Urban: final `|k̂ − 1.02| ≤ 0.015`, seeds 1–5. (Box range `1.010–1.025`.)
12. Matched Monte Carlo (N = 50): position NEES time-mean inside `[1.484439, 2.591224]` with at least 80% of epochs inside; full NEES time-mean inside `[4.161956, 5.913773]` with at least 70% of epochs inside. (Box over 4 blocks of 50 seeds: position 92–99%, full 83–94%.)
13. Urban Monte Carlo (N = 50, bias model on): position NEES time-mean inside `[1.484439, 2.591224]` with at least 80% of epochs inside. Full NEES is reported as a diagnostic, not gated. (Box: position 94–100%; full 64–100%, which is why it is not gated.)
14. Ablation `urban_white_model` (N = 50): position NEES time-mean `> 5.18` (twice the upper bound). This proves the consistency test can fail. (Box: `20.8–26.6`.)
15. Reacquisition: in the matched scenario, injecting `+20 m` east into the state at `t = 30 s` without touching `P` gives error `< 2 m` within `5 s`, with `reacquisitions ≥ 1`. Unmodified matched and urban runs, seeds 1–50, have `reacquisitions == 0`. (Box: recovered in `1.8–2.1 s`; zero reacquisitions on unmodified runs.)
16. Matched Monte Carlo: the aggregate pre-gate GNSS rejection rate is within `[0.5%, 2.0%]` (nominal 1%). Urban Monte Carlo outlier recall `≥ 95%` and inlier false-rejection rate `≤ 2.0%` (scored with labels). (Box: matched rejection rate 0.96%; urban recall 100%, false rejection 1.06%.)
17. Determinism holds as specified (in-process bit-identical, `-O0`/`-O2` within `1e-9 m`, WASM vs native within `1e-6 m` with identical gate decisions).
18. `npm ci && npx tsc --noEmit && npx vitest run && npm run build` pass in `web/`; Playwright passes the assertions above and writes `docs/demo.png`; the Pages workflow deploys the tested `dist` artifact from `main` only after the core and web workflows succeed.
19. The optional Humble workflow, when run, produces the comparison CSV meeting the coverage check. The README does not claim our filter beats `robot_localization`.
20. `LICENSE` is byte-identical to https://www.apache.org/licenses/LICENSE-2.0.txt (SHA-256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`). `NOTICE` starts `ekf-gnss-localization` / `Copyright 2026 Arpan Bom` and lists build/runtime third-party components (GoogleTest, nlohmann/json, Emscripten runtime in the WASM bundle). Authored files carry `SPDX-License-Identifier: Apache-2.0`.
21. README: simulation only; no Autoware, RTK, real receiver, real vehicle or certification claims; states the observability limits, the 8-state choice and the ablation result; names the four applycoding repos it is not; credits idea references from REFERENCES.md; includes a "How this was verified" section listing the tests behind criteria 2–17.
22. `git log --format='%an <%ae> | %cn <%ce>%n%b'` shows only `Arpan Bom <applycoding@gmail.com>` for author and committer, with no `Co-authored-by` lines and no AI tool mentions.
23. `tools/check_no_heap.py` passes and the operator-new-abort test passes.

## Out of scope

RTK, carrier phase, raw pseudoranges, tightly coupled GNSS/INS, 3D or 6-DoF INS (roll, pitch), lateral sideslip, accelerometer bias, lever arm, LiDAR or map matching, Autoware integration, CAN, HILS, real vehicles and receivers, retrodiction and out-of-order smoothing, `navsat_transform_node`, robust/soft reweighting, reverse-driving initialization, Jazzy and Windows CI, in-browser Monte Carlo, and copying any third-party code.

## Documentation and honesty

README must state: simulation only; scripted unicycle truth (not a vehicle dynamics model); heading unobservable at rest and initialized from GNSS motion; first-order `Q` validated at 100 Hz only; truth gyro bias is constant while the filter allows a small walk; the ablation shows the white-noise model is overconfident; NEES bounds are engineering regression checks; `robot_localization` is a same-information side-by-side, not a benchmark to beat. All thresholds are written as numbers with the box-measured margins, not "tuned until green".

## License

Apache-2.0 (`LICENSE` verbatim, `NOTICE`, SPDX headers). Third-party licences stay with their fetched trees; the distributed WASM bundle's Emscripten runtime notices are included in `web/dist` (e.g. `third_party_licenses.txt`).

## Build and CI

```text
cmake --preset default && cmake --build --preset default && ctest --preset default   # core (required)
python3 tools/check_no_heap.py
cmake --preset wasm   # sources pinned emsdk 7.0.0, builds ekf_core+ekf_sim to web/public/
cd web && npm ci && npx tsc --noEmit && npx vitest run && npm run build && npx playwright test
```

- `core.yml` (required): build, ctest including the `mc` label, no-heap check, determinism `-O0`/`-O2`.
- `web.yml` (required): WASM build, typecheck, Vitest, Vite build, Playwright, upload `demo.png` and `dist` artifact.
- `pages.yml`: on `main` after `core` and `web` succeed, deploy the tested `dist` artifact. Permissions `pages: write`, `id-token: write`; concurrency group `pages`. Never runs on PR code.
- `ros2-humble.yml` (optional, manual + weekly): pinned image digest, apt install, colcon build of `ros2/ekf_gnss_demo`, comparison launch, upload CSV.
- No timing goldens.

## Risks

- The first-order `Q` and Euler predict are validated only at `dt = 0.01 s`; hence the `dt ≤ 0.02 s` contract.
- The C++ RNG produces different draws from the Python prototype. Every threshold was set against the worst of 200 prototype seeds with margin, but if a criterion fails for a real implementation reason, fix the code. Never move a threshold to make CI green; record any justified change in `DEVIATIONS.md` with the measured data.
- Urban full NEES varies a lot between seed blocks (64–100% of epochs inside), so it is diagnostic only.
- Gate-on Monte Carlo includes selection effects; the README notes it.
- WASM vs native libm differences could flip a gate decision near the threshold; the matched scenario keeps this unlikely, and the test will show it if it happens.
- `robot_localization` uses a different state model; its numbers are illustrative.
- Humble reaches end of life in May 2027.

## Verification already done (2026-11-10, on the box)

Scripts are in `_proto/` next to this plan (not part of the repo). Python 3 + NumPy 2.5.4 + SciPy 1.18.1, and pymap3d 3.x in a scratch venv for geodesy cross-checks.

- Chi-square constants: 2-DOF 99% gate `9.210340371976184 = −2 ln 0.01`; 1-DOF 99% `6.634896601021214`; N = 50 bounds df 100 `[1.484439, 2.591224]`, df 250 `[4.161956, 5.913773]`; `χ²_{0.9973}(2) = 11.829007`.
- Exact 2-state example (Python `fractions`) and the 8-state frozen example (NumPy) as listed above. Joseph form equals `(I − KH)P` at the optimal gain to 0.0 difference; `P` stays positive definite.
- Jacobian vs central differences: max difference `1.4e-10`.
- Geodesy: forward equations match pymap3d within `1e-9 m`; one-step Bowring inverse round-trip worst `1.03e-6 m` (fails `1e-6`); iterative inverse worst `3.7e-9 m`.
- Prototype sim with this exact trajectory, sensor model, init, gates and recovery: 4 blocks × 50 seeds per scenario. Results are quoted in each criterion. Findings that changed the design: naive first-fix heading init diverged under multipath (fixed by the robust median-window init); a white-noise GNSS model is overconfident under Gauss–Markov error (NEES 21–27, fixed by augmenting `β`); a wheel lockout window equal to the slip length broke consistency (fixed with 2 s); the `p`–`β` initial cross-covariance improved urban consistency; gating on/off barely matters for the 5-state inflated-R filter (ratio ≈ 0.98), so the draft's `≤ 0.85×` gating criterion would only hold for the 8-state design, where gate-off diverges (≈ 16–20 m RMSE).
- Pins: GoogleTest, pybind11, nlohmann/json and emsdk tags resolve to the listed SHAs (`gh api`); `npm view` gives typescript 7.0.2, vite 8.3.7, vitest 5.1.0 and @playwright/test 1.65.0; PyPI pytest 9.2.0; Node 26.13.0 is the newest LTS (Lithium); rosdistro Humble lists robot_localization 3.5.5-1.
