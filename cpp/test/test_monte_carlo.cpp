// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <gtest/gtest.h>

TEST(MonteCarlo, MatchedSeedBlock) {
  double mean_rmse = 0.0;
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    const auto r = ekf::sim::RunScenario("matched", seed);
    mean_rmse += r.metrics.rmse_horizontal_m;
  }
  mean_rmse /= 5.0;
  EXPECT_LT(mean_rmse, 1.5);
}
