# Verification map

| # | Criterion | ctest / CI evidence |
| --- | --- | --- |
| 1 | Core build + `ctest` | `core.yml`; `ctest --preset default` |
| 2 | 2-state + 8-state frozen | `Exact2State.PredictUpdateJoseph`; `FilterFrozen.PredictWheelGnss` |
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
| 15 | Reacquisition | `Reacquisition.MatchedInjectEast20m`, `Reacquisition.UnmodifiedSeedsHaveZeroReacquisitions` |
| 16 | Rejection / recall | `MonteCarlo.MatchedGnssRejectionRateN50`, `MonteCarlo.UrbanOutlierRecallAndFalseRejectN50` |
| 17 | Determinism | `Determinism.SameSeedBitIdenticalTrace`, `Determinism.O0O2Parity`; `Determinism.WasmMatchesNativeParity` when `web/public/ekf.wasm` exists |
| 18 | Web + Pages gate | `web.yml`; Vitest `web/tests/smoke.test.ts`; Playwright `e2e/demo.spec.ts`; `pages.yml` checks green `core` on same SHA |
| 19 | ROS Humble | `ros2-humble.yml` (placeholder); see `DEVIATIONS.md` |
| 20 | LICENSE / NOTICE / SPDX | `LICENSE` SHA-256; `NOTICE`; SPDX headers |
| 21 | README honesty | `README.md` |
| 22 | Git authorship | branch `git log` |
| 23 | No heap | `tools/check_no_heap.py`; `NoHeapOperator.FilterStepsWithoutAlloc` |
| — | Gate thresholds | `Gates.WheelThreshold`, `Gates.GnssThreshold` |
