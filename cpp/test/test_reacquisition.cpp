// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

double horizontal_error_at(const ekf::sim::Trace& trace, double t_s) {
  for (const auto& tk : trace.ticks) {
    if (std::abs(tk.t_s - t_s) < 0.02 && tk.filter_initialized) {
      return std::hypot(tk.est_e_m - tk.truth_e_m, tk.est_n_m - tk.truth_n_m);
    }
  }
  return 1e9;
}

}  // namespace

TEST(Reacquisition, MatchedInjectEast20m) {
  ekf::sim::ScenarioSpec spec = ekf::sim::ScenarioMatched();
  spec.inject_time_s = 30.0;
  spec.inject_e_m = 20.0;
  const auto result = ekf::sim::RunScenario(spec, 1U);
  EXPECT_GE(result.metrics.reacquisitions, 1U);
  double min_err = 1e9;
  for (double t = 30.0; t <= 35.0 + 1e-9; t += 0.1) {
    min_err = std::min(min_err, horizontal_error_at(result.trace, t));
  }
  EXPECT_LT(min_err, 2.0);
}

TEST(Reacquisition, UnmodifiedSeedsHaveZeroReacquisitions) {
  for (const char* name : {"matched", "urban"}) {
    for (std::uint64_t seed = 1; seed <= 50; ++seed) {
      const auto r = ekf::sim::RunScenario(name, seed);
      EXPECT_EQ(r.metrics.reacquisitions, 0U) << name << " seed " << seed;
    }
  }
}
