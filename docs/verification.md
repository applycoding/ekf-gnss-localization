# Verification map

| # | Criterion | Tests / evidence |
| --- | --- | --- |
| 1 | Core configure/build/ctest | `core.yml`, `ctest --preset default` |
| 2 | 2/8-state examples | `FilterFrozen.PredictWheelGnss` (predict + wheel + GNSS golden) |
| 3 | Jacobian FD | planned in `test_unit` |
| 4 | Geodesy | planned in `test_unit` |
| 5 | Invalid inputs | planned in `test_unit` |
| 6 | Matched RMSE | `Scenario.MatchedCriteriaSeeds1to5` |
| 7 | Urban RMSE | `Scenario.UrbanCriteriaSeeds1to5` |
| 8 | Urban dropout | `Scenario.UrbanCriteriaSeeds1to5` |
| 9 | DR vs EKF @ 90s | `Scenario.UrbanCriteriaSeeds1to5`, matched |
| 10 | Gyro bias | `Scenario.MatchedCriteriaSeeds1to5` |
| 11 | Wheel scale | `Scenario.UrbanCriteriaSeeds1to5` |
| 12 | Matched MC NEES | `MonteCarlo.MatchedPositionNeessN50` |
| 13 | Urban MC NEES | `MonteCarlo.UrbanPositionNeessN50` |
| 14 | White-model ablation | `MonteCarlo.UrbanWhiteModelFailsNeessN50` |
| 15 | Reacquisition | not yet automated |
| 16 | Rejection rates | not yet automated |
| 17 | Determinism | `Determinism.SameSeedSameRmse`, `core.yml` O0/O2 job |
| 18 | Web CI | `web.yml`, Playwright `e2e/demo.spec.ts`, `docs/demo.png` |
| 19 | ROS Humble | optional workflow not included in this branch |
| 20 | License/NOTICE/SPDX | `LICENSE` hash, `NOTICE`, headers |
| 21 | README honesty | `README.md` |
| 22 | Git authorship | `git log` on branch |
| 23 | No heap | `tools/check_no_heap.py`, `NoHeapOperator.FilterReset` |
