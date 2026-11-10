// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <gtest/gtest.h>

TEST(Scenario, MatchedRunsAndInitializes) {
  const auto result = ekf::sim::RunScenario("matched", 1U);
  EXPECT_GT(result.metrics.init_time_s, 5.0);
  EXPECT_LT(result.metrics.init_time_s, 14.0);
  EXPECT_LT(result.metrics.rmse_horizontal_m, 2.0);
}
