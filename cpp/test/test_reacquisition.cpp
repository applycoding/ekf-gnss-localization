// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

constexpr double kInjectTimeS = 30.0;
constexpr double kAssertTimeS = 35.0;
constexpr int kAssertTickIndex = 3500;

double horizontal_error_at_index(const ekf::sim::Trace& trace, std::size_t idx) {
  if (idx >= trace.ticks.size()) {
    return 1e9;
  }
  const auto& tk = trace.ticks[idx];
  if (!tk.filter_initialized) {
    return 1e9;
  }
  return std::hypot(tk.est_e_m - tk.truth_e_m, tk.est_n_m - tk.truth_n_m);
}

}  // namespace

TEST(Reacquisition, MatchedInjectEast20m) {
  for (std::uint64_t seed = 1; seed <= 5; ++seed) {
    ekf::sim::ScenarioSpec spec = ekf::sim::ScenarioMatched();
    spec.inject_time_s = kInjectTimeS;
    spec.inject_e_m = 20.0;
    const auto result = ekf::sim::RunScenario(spec, seed);
    EXPECT_GE(result.metrics.reacquisitions, 1U) << "seed " << seed;
    const double err =
        horizontal_error_at_index(result.trace, static_cast<std::size_t>(kAssertTickIndex));
    EXPECT_LT(err, 2.0) << "seed " << seed << " horizontal error at t=" << kAssertTimeS << " s";
  }
}

TEST(Reacquisition, UnmodifiedSeedsHaveZeroReacquisitions) {
  for (const char* name : {"matched", "urban"}) {
    for (std::uint64_t seed = 1; seed <= 50; ++seed) {
      const auto r = ekf::sim::RunScenario(name, seed);
      EXPECT_EQ(r.metrics.reacquisitions, 0U) << name << " seed " << seed;
    }
  }
}
