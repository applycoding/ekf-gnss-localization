// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <gtest/gtest.h>

namespace {

void expect_matched(std::uint64_t seed) {
  const auto r = ekf::sim::RunScenario("matched", seed);
  EXPECT_LT(r.metrics.rmse_horizontal_m, 1.0);
  EXPECT_LT(r.metrics.rmse_horizontal_m, 0.5 * r.metrics.baseline_gnss_only.rmse_horizontal_m);
  EXPECT_LT(r.metrics.gyro_bias_error_at_40s_rad_s, 0.004);
  EXPECT_GT(r.metrics.dead_reckoning_error_at_90s_m, 20.0);
  EXPECT_LE(r.metrics.ekf_error_at_90s_m, 4.0);
}

void expect_urban(std::uint64_t seed) {
  const auto r = ekf::sim::RunScenario("urban", seed);
  EXPECT_LT(r.metrics.rmse_horizontal_m, 4.0);
  EXPECT_LT(r.metrics.rmse_horizontal_m, 0.5 * r.metrics.baseline_ekf_gate_off.rmse_horizontal_m);
  EXPECT_LE(r.metrics.dropout_max_m, 8.0);
  EXPECT_LE(r.metrics.error_after_dropout_return_m, 5.0);
  EXPECT_GT(r.metrics.dead_reckoning_error_at_90s_m, 20.0);
  EXPECT_LE(r.metrics.ekf_error_at_90s_m, 4.0);
  EXPECT_LE(std::abs(r.metrics.final_wheel_scale - 1.02), 0.015);
}

}  // namespace

TEST(Scenario, MatchedCriteriaSeeds1to5) {
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    expect_matched(seed);
  }
}

TEST(Scenario, UrbanCriteriaSeeds1to5) {
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    expect_urban(seed);
  }
}

TEST(Scenario, MatchedGyroBiasThreeValuesSeeds1to5) {
  constexpr double kBiases[] = {-0.01, 0.0, 0.02};
  for (double bg : kBiases) {
    for (std::uint64_t seed = 1; seed <= 5; ++seed) {
      ekf::sim::ScenarioSpec spec = ekf::sim::ScenarioMatched();
      spec.gyro_bias_rad_s = bg;
      const auto r = ekf::sim::RunScenario(spec, seed);
      EXPECT_LT(r.metrics.gyro_bias_error_at_40s_rad_s, 0.004)
          << "bias=" << bg << " seed=" << seed;
    }
  }
}

TEST(Scenario, MatchedGyroBiasMutantFailsFor01) {
  ekf::sim::ScenarioSpec spec = ekf::sim::ScenarioMatched();
  spec.gyro_bias_rad_s = 0.01;
  spec.filter.estimate_gyro_bias = false;
  const auto r = ekf::sim::RunScenario(spec, 1U);
  EXPECT_GE(r.metrics.gyro_bias_error_at_40s_rad_s, 0.004);
}
