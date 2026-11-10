// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <gtest/gtest.h>

TEST(Determinism, SameSeedSameRmse) {
  const auto a = ekf::sim::RunScenario("matched", 7U);
  const auto b = ekf::sim::RunScenario("matched", 7U);
  EXPECT_DOUBLE_EQ(a.metrics.rmse_horizontal_m, b.metrics.rmse_horizontal_m);
}
