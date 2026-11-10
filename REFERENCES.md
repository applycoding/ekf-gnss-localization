# References — ekf-gnss-localization

These permissively licensed projects are **references only**. No source, test, figure or text is copied, translated or vendored. Ideas that shaped a file get a one-line credit in the README and, where useful, a comment pointing at the reference.

Licences were checked on 2026-11-10 against each project's actual licence file (raw URL below). GitHub's SPDX field was used only as a hint, because it reads `NOASSERTION` for several of them. Revisions are the default-branch HEAD on that date.

**Outbound licence for this repo: Apache-2.0** (verbatim `LICENSE`, `NOTICE`, SPDX headers).

## Not a duplicate of these applycoding repos

| Repo | Licence (`gh repo list`) | Difference |
| --- | --- | --- |
| [applycoding/cpp-flight-controller](https://github.com/applycoding/cpp-flight-controller) | MIT | Quadrotor attitude control and attitude filters; this repo is planar GNSS/odometry/IMU localization. |
| [applycoding/ros2-nav2-slam-sim](https://github.com/applycoding/ros2-nav2-slam-sim) | MIT | Gazebo + slam_toolbox + Nav2; no GNSS fusion. |
| [applycoding/ros2-vision-tracking-demo](https://github.com/applycoding/ros2-vision-tracking-demo) | Apache-2.0 | Kalman object tracker on detections; not ego localization. |
| [applycoding/sim-gimbal-stabilizer](https://github.com/applycoding/sim-gimbal-stabilizer) | Apache-2.0 | Gimbal attitude ESKF; different problem. |

## Primary idea references (all MIT, BSD or Apache)

### 1. cra-ros-pkg/robot_localization

- URL: https://github.com/cra-ros-pkg/robot_localization (branch `humble-devel`, HEAD `7ebaee2b7b20`, 2026-11-02)
- Licence: **BSD-3-Clause** style, with two notices: Charles River Analytics (2014–2016) and a modified BSD notice from Austin Robot Technology / UT Austin. Raw: https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/humble-devel/LICENSE
- Note: `package.xml` on `humble-devel` declares `Apache License 2.0`, which does not match the LICENSE file. We only use it as an apt runtime dependency in the optional ROS job, so either way nothing is redistributed. Rosdistro Humble release: 3.5.5-1.
- Idea: the `ekf_node` sensor configuration vector, `two_d_mode`, Mahalanobis rejection thresholds, and the REP-105 frame conventions for the side-by-side comparison. Not copied; our filter is a different, smaller model.

### 2. AtsushiSakai/PythonRobotics

- URL: https://github.com/AtsushiSakai/PythonRobotics (HEAD `ef64608ceb00`, 2026-11-10), section `Localization/extended_kalman_filter`
- Licence: **MIT** ("The MIT License (MIT), Copyright (c) 2016 - now Atsushi Sakai and other contributors"). Raw: https://raw.githubusercontent.com/AtsushiSakai/PythonRobotics/master/LICENSE
- Idea: the general shape of a teaching EKF localization demo (truth vs dead reckoning vs EKF plot). Our state, noise models, gating and metrics are different and written here.

### 3. simondlevy/TinyEKF

- URL: https://github.com/simondlevy/TinyEKF (HEAD `ba0d2a90e22f`, 2026-04-09)
- Licence: **MIT** (file is `LICENSE.md`, "Copyright (c) Simon D. Levy … MIT License"). Raw: https://raw.githubusercontent.com/simondlevy/TinyEKF/master/LICENSE.md
- Idea: static-allocation EKF suitable for small targets, which motivates our no-heap core. We write our own fixed-size matrices and Joseph-form update.

### 4. geographiclib/geographiclib (C++)

- URL: https://github.com/geographiclib/geographiclib (HEAD `48959dfc4a58`, 2025-11-14)
- Licence: **MIT** ("The MIT License (MIT). Copyright (c) 2008-2025, Charles Karney"). Raw: https://raw.githubusercontent.com/geographiclib/geographiclib/main/LICENSE.txt
- Idea: WGS84 constants and geocentric/local-cartesian conventions, used only to sanity-check our test vectors. Our iterative inverse is written from the textbook equations.

### 5. geospace-code/pymap3d

- URL: https://github.com/geospace-code/pymap3d (HEAD `033895e2f1cf`, 2026-06-22)
- Licence: **BSD-2-Clause** (SciVision, Nievinski, Kleder notices). Raw: https://raw.githubusercontent.com/geospace-code/pymap3d/main/LICENSE
- Use: an independent oracle during planning. The geodesy vectors in PLAN.md agree with pymap3d 3.x within 1e-9 m. The repo may use it as an optional dev-only check in `python/requirements-dev.txt`. It is never a runtime dependency.

### 6. rlabbe/filterpy

- URL: https://github.com/rlabbe/filterpy (HEAD `5df4e095fac0`, 2026-10-27)
- Licence: **MIT** ("Copyright (c) 2015 Roger R. Labbe Jr"). Raw: https://raw.githubusercontent.com/rlabbe/filterpy/master/LICENSE
- Idea: NEES/NIS helper conventions and Joseph-form presentation. Not a dependency.

### 7. autowarefoundation/autoware_core — `localization/autoware_ekf_localizer`

- URL: https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ekf_localizer (HEAD `02d10f9299fa`, 2026-11-10)
- Licence: **Apache-2.0** (root LICENSE). Raw: https://raw.githubusercontent.com/autowarefoundation/autoware_core/main/LICENSE
- Idea only: Mahalanobis gating and yaw-bias handling, for vocabulary when discussing the TIER IV context. **Not a dependency, not copied, and the README must not claim Autoware experience or integration.**

## Background reading (not code)

- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001 — the filter-consistency chapter (NEES/NIS chi-square tests). The maths is paraphrased in our own words; no text or figures are copied.
- R. Labbe, *Kalman and Bayesian Filters in Python* (https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python). Its `license.html` text and link disagree (BY-NC-SA 4.0 text vs a BY 4.0 link), so it is reading only. Nothing is reused.

## Looked at, excluded as code references

| Project | Licence (from its own file) | Rule |
| --- | --- | --- |
| [tomojitakasu/RTKLIB](https://github.com/tomojitakasu/RTKLIB) | BSD-2-Clause plus extra clauses for bundled Windows binaries, per `readme.txt` (no LICENSE file); versions ≤ 2.4.1 were GPLv3 | Idea-only. Not needed for a position-domain demo. |
| [gnss-sdr/gnss-sdr](https://github.com/gnss-sdr/gnss-sdr) | GPL-3.0 (`COPYING`) | **Idea-only.** Never read to implement anything here. |
| Eigen 3.4 | MPL-2.0 | Not used. Hand-written 8×8 fixed-size maths instead. |

## Build and runtime dependencies (not references)

GoogleTest 1.18.0 (BSD-3-Clause), nlohmann/json 3.12.0 (MIT), pybind11 3.1.0 (BSD-3-Clause, optional), Emscripten / emsdk 7.0.0 (MIT or NCSA; its runtime ends up in the WASM bundle, so its notices ship in `web/dist`), TypeScript (Apache-2.0), Vite / Vitest (MIT), Playwright (Apache-2.0), ROS 2 Humble packages (Apache-2.0, optional job only). `NOTICE` lists them.

## Licence compatibility with Apache-2.0 outbound

| Inbound | Compatible | Ours |
| --- | --- | --- |
| Apache-2.0 | Yes | autoware_core (idea only) |
| MIT | Yes (keep notice if code were ever reused) | PythonRobotics, TinyEKF, GeographicLib, filterpy |
| BSD-2 / BSD-3 | Yes (keep notice and disclaimer if code were ever reused) | robot_localization, pymap3d |
| GPL-3.0 | No for derived code | gnss-sdr: idea-only |

All seven primary references are MIT, BSD or Apache-2.0. None is GPL or LGPL.
