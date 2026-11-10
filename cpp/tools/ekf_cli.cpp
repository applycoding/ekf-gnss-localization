// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  const char* scenario = "matched";
  std::uint64_t seed = 1;
  if (argc > 1) {
    scenario = argv[1];
  }
  if (argc > 2) {
    seed = static_cast<std::uint64_t>(std::strtoull(argv[2], nullptr, 10));
  }
  const ekf::sim::RunResult result = ekf::sim::RunScenario(scenario, seed);
  const auto& last = result.trace.ticks.back();
  std::cout << "scenario=" << result.trace.scenario_name << " seed=" << seed
            << " init_t_s=" << result.metrics.init_time_s
            << " rmse_m=" << result.metrics.rmse_horizontal_m << " gnss_acc=" << result.metrics.gnss_accepted
            << " gnss_gate=" << result.metrics.gnss_gated << " truth_e=" << last.truth_e_m
            << " est_e=" << last.est_e_m << "\n";
  return 0;
}
