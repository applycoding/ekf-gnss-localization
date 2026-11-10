# Build tasks — ekf-gnss-localization

Execute in order on a feature branch. Follow PLAN.md exactly. Deviate only if a pin or a test proves a step broken; record the reason and the measured data in `DEVIATIONS.md`. **Never weaken a threshold or test to make CI green.**

Every commit: author and committer **Arpan Bom <applycoding@gmail.com>**. No `Co-authored-by` trailers. No mention of AI tools anywhere in the repo or commit messages. Licence **Apache-2.0**. Simulation only.

A task is done only when its **Accept** checks pass.

## 1. Skeleton, licence, presets

- Tree from PLAN.md "File layout". `LICENSE` byte-identical to https://www.apache.org/licenses/LICENSE-2.0.txt (SHA-256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`). `NOTICE` starting `ekf-gnss-localization` / `Copyright 2026 Arpan Bom`. README stub saying simulation-only portfolio demo, Apache-2.0.
- Root `CMakeLists.txt`: C++17, `CMAKE_CXX_EXTENSIONS OFF`, `-ffp-contract=off`, warnings as errors in core (`-Wall -Wextra -Wpedantic -Werror`), options as in PLAN.md. `CMakePresets.json` with `default` and `wasm`.
- SPDX header on every authored file.

**Accept:** `sha256sum LICENSE` matches; `cmake --preset default` configures with no Python/Node/ROS lookup.

## 2. Pins and dependencies

- GoogleTest via FetchContent at the full SHA. nlohmann/json only for `ekf_sim`/CLI. pybind11 only if `EKF_BUILD_PYTHON`.
- `.github/workflow-pins.md` lists every Action with its full SHA and version comment.

**Accept:** a clean configure fetches exactly the pinned commits; json is not reachable from core headers (`check_no_heap.py` also greps for `nlohmann`).

## 3. Core maths and geodesy

- `mat.hpp` fixed-size matrices; 1×1 / 2×2 / 5×5 Cholesky solves.
- `WrapPi`, WGS84 forward (closed form) and **iterative** inverse, ENU from origin.

**Accept:** criterion 4 tests (anchors, Tokyo and 45/45/1000 vectors, ENU basis, 1000 round-trips) pass.

## 4. Filter

- 8-state predict (Euler), `F`, `Q`, `dt` contract, wheel and GNSS updates in Joseph form, Cholesky on `S`, finite/PD checks with rollback, both chi-square gates, GNSS reacquisition with the consistency check, wheel lockout, config flags `model_gnss_bias` / `estimate_wheel_scale` / `gate`.
- `UpdateResult` with pre-gate NIS.

**Accept:** criterion 2 (2-state exact, 8-state frozen example), criterion 3 (Jacobian), criterion 5 (invalid inputs bit-identical), gate threshold tests.

## 5. Initializer

- `NoFix → Collecting → Initialized`, windows A/B of 5 fixes, 20 m odometry distance, straightness check `< 0.1 rad`, forward speed only, medians, `P₀` including the `p`–`β` cross-covariance.

**Accept:** deterministic initializer tests from PLAN.md (outliers in both windows, turning window restarts, non-positive speed never initializes).

## 6. Simulator

- xoshiro256** + splitmix64 RNG, own Box–Muller, independent named streams. Exact trajectory equations (midpoint truth integration). Sensor, slip, Gauss–Markov, dropout and multipath models. Integer-tick scheduler (predict → wheel → GNSS). Scenarios `matched`, `urban`, `urban_white_model` as JSON. Baselines (a)–(d). Metrics: RMSE, dropout metrics, marginal position NEES, 5-state NEES, pre-gate NIS, recall / false rejection, reacquisitions.
- `ekf_cli` writes CSV/JSON traces.

**Accept:** criteria 6–11 and 15 pass for seeds 1–5; the RNG unit test shows the same stream values on two builds.

## 7. Monte Carlo tests

- ctest label `mc`: N = 50 seeds; frozen bounds `[1.484439, 2.591224]` (position) and `[4.161956, 5.913773]` (full).

**Accept:** criteria 12, 13, 14 and 16 pass. Print time-means, epoch fractions and upper/lower violations in the test log.

## 8. No-heap and determinism

- `tools/check_no_heap.py`; operator-new-abort test binary (plain, array, aligned, nothrow).
- Determinism tests (`-O0` vs `-O2` job, in-process repeat).

**Accept:** criteria 17 (native parts) and 23.

## 9. Core CI

- `core.yml` on `ubuntu-24.04` with `g++-13`: configure, build, ctest (including `mc`), no-heap check, `-O0`/`-O2` determinism. Log compiler and CMake versions.

**Accept:** criterion 1; the workflow run on the branch is **completed** and green.

## 10. WASM and web

- `wasm` preset with emsdk 7.0.0 (install + activate 7.0.0, log `emcc --version`, re-assert Node 26). Single-threaded. Small ABI returning typed arrays.
- Vite + TS app, base `/ekf-gnss-localization/`, Canvas 2D plots and controls from PLAN.md, model-mismatch label, single-run NEES bounds, Run enabled after module load.
- Vitest unit tests for the UI glue. Playwright test as in PLAN.md, single worker, writes repo-root `docs/demo.png`. WASM vs native comparison test on the full matched run (export the native trace from `ekf_cli`).
- `web.yml`: WASM build, `npm ci`, `tsc --noEmit`, Vitest, build, Playwright, upload `demo.png` and `dist`.

**Accept:** criteria 17 (WASM part) and 18 (web part); look at `docs/demo.png` and confirm the plots show data.

## 11. Pages

- `pages.yml`: deploys the tested `dist` artifact from `main` only after `core` and `web` succeed; `pages: write`, `id-token: write`, concurrency `pages`; never on PR code.
- Commit `docs/demo.png` once by hand (CI does not auto-commit).

**Accept:** the workflow file is valid and gated as described. (Pages is enabled by the repo owner after merge.)

## 12. Optional Humble comparison

- `ros2/ekf_gnss_demo`: sim node, our filter node, launch with `robot_localization` `ekf_node` configured as in PLAN.md (same information, no `navsat_transform_node`, `publish_tf: false`), static transforms, `/clock`, CSV recorder.
- `ros2-humble.yml`: `ros:humble-ros-base-jammy` pinned by digest, manual + weekly trigger, visible status, not a required check.

**Accept:** criterion 19 when run; core stays green without it.

## 13. Optional Python

- pybind11 module + `python/bench/monte_carlo.py` producing `docs/figures/*.png` (NEES vs bounds, error vs time, ablation). Pinned, hashed `requirements-dev.txt`.

**Accept:** builds only when `EKF_BUILD_PYTHON=ON`; core unaffected.

## 14. Docs and honesty

- README per PLAN.md "Documentation and honesty", including the "How this was verified" section, observability limits, the ablation result, the four repos it is not, idea credits from REFERENCES.md, Humble end of life (May 2027).
- `docs/verification.md` with test names mapped to criteria 1–23.

**Accept:** criteria 20 and 21; a case-insensitive search of the tree (excluding `node_modules`) for coding-assistant names, "generated by" and `co-authored` finds nothing relevant.

## 15. PR

- One PR from the branch to `main`, description listing every criterion with its test name or evidence and CI run URLs. Final CI run on the PR head must be **completed and green**.

**Accept:** criterion 22 (`git log` check) and all required checks green.
