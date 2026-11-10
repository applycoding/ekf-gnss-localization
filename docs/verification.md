# Verification map

| # | Criterion | ctest / CI evidence |
| --- | --- | --- |
| 1 | Core build + `ctest` | `core.yml`; `ctest --preset default` |
| 2 | 2-state + 8-state frozen | `Exact2State.PredictUpdateJoseph` (checks the small `ekf::Linear2Predict` / `Linear2JosephUpdate` helpers, which the 8-state filter does not call); `FilterFrozen.PredictWheelGnss` (the production 8-state predict, wheel and GNSS path) |
| 3 | Jacobian FD | `Jacobian.CentralDifferenceFiftyStates` |
| 4 | Geodesy anchors + round-trips + ENU basis | `Geodesy.*` |
| 5 | Invalid / numerical backup | `InvalidInputs.*` |
| 6 | Matched RMSE | `Scenario.MatchedCriteriaSeeds1to5` |
| 7 | Urban gated RMSE | `Scenario.UrbanCriteriaSeeds1to5` |
| 8 | Urban dropout | `Scenario.UrbanCriteriaSeeds1to5` |
| 9 | DR vs EKF @ 90 s (both scenarios) | `Scenario.MatchedCriteriaSeeds1to5`, `Scenario.UrbanCriteriaSeeds1to5` |
| 10 | Gyro bias + mutant | `Scenario.MatchedGyroBiasThreeValuesSeeds1to5`, `Scenario.MatchedGyroBiasMutantFailsFor01` |
| 11 | Wheel scale urban | `Scenario.UrbanCriteriaSeeds1to5` |
| 12 | Matched MC position + full NEES | `MonteCarlo.MatchedPositionNeessN50`, `MonteCarlo.MatchedFullNeessN50` |
| 13 | Urban MC position NEES | `MonteCarlo.UrbanPositionNeessN50` |
| 14 | White-model ablation | `MonteCarlo.UrbanWhiteModelFailsNeessN50` |
| 15 | Reacquisition | `Reacquisition.MatchedInjectEast20m` (seeds 1–5, error at **t = 35 s**, `reacquisitions ≥ 1`); `Reacquisition.UnmodifiedSeedsHaveZeroReacquisitions` |
| 16 | Rejection / recall | `MonteCarlo.MatchedGnssRejectionRateN50`, `MonteCarlo.UrbanOutlierRecallAndFalseRejectN50` |
| 17 | Determinism | `Determinism.SameSeedBitIdenticalTrace`, `Determinism.O0O2Parity`; `Determinism.WasmMatchesNativeParity` (repo-root `WORKING_DIRECTORY`, `EKF_REQUIRE_WASM_PARITY=1` in `core.yml`; `node tools/wasm_native_parity.mjs` in `web.yml`). The WASM check compares positions within 1e-6 m on matched seed 1; it does not yet compare gate decisions |
| 18 | Web + Pages gate | `web.yml`; Vitest `neesSummaryLabel` in `web/tests/smoke.test.ts`; Playwright `e2e/demo.spec.ts`; `pages.yml` on `core`/`web` `workflow_run`, deploy when both succeeded on same SHA |
| 19 | ROS Humble | `ros2-humble.yml` (placeholder); see `DEVIATIONS.md` |
| 20 | LICENSE / NOTICE / SPDX | `LICENSE` SHA-256; `NOTICE`; SPDX headers |
| 21 | README honesty | `README.md` |
| 22 | Git authorship | branch `git log` |
| 23 | No heap | `tools/check_no_heap.py`; `NoHeapOperator.FilterStepsWithoutAlloc` |
| — | Gate thresholds | `Gates.WheelThreshold`, `Gates.GnssThreshold` |
