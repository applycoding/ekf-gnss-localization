// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  const char* scenario = "matched";
  std::uint64_t seed = 1;
  bool parity = false;
  if (argc > 1) {
    scenario = argv[1];
  }
  if (argc > 2) {
    if (std::strcmp(argv[2], "--parity") == 0) {
      parity = true;
    } else {
      seed = static_cast<std::uint64_t>(std::strtoull(argv[2], nullptr, 10));
    }
  }
  if (argc > 3 && std::strcmp(argv[3], "--parity") == 0) {
    parity = true;
  }
  const ekf::sim::RunResult result = ekf::sim::RunScenario(scenario, seed);
  if (parity) {
    std::cout << std::setprecision(17);
    for (const auto& t : result.trace.ticks) {
      if (!t.filter_initialized) {
        continue;
      }
      if (std::fmod(t.t_s + 1e-9, 1.0) > 0.02) {
        continue;
      }
      std::cout << t.t_s << " " << t.est_e_m << " " << t.est_n_m << " "
                << static_cast<unsigned>(result.metrics.gnss_gated) << " "
                << static_cast<unsigned>(result.metrics.gnss_accepted) << "\n";
    }
    return 0;
  }
  const auto& last = result.trace.ticks.back();
  double nees_sum = 0.0;
  int nees_n = 0;
  for (const auto& g : result.trace.gnss) {
    if (g.accepted && g.nees_pos > 0.0 && g.t_s >= result.metrics.init_time_s + 5.0) {
      nees_sum += g.nees_pos;
      ++nees_n;
    }
  }
  const double nees_mean = nees_n > 0 ? nees_sum / nees_n : 0.0;
  std::cout << "scenario=" << result.trace.scenario_name << " seed=" << seed
            << " init_t_s=" << result.metrics.init_time_s
            << " rmse_m=" << result.metrics.rmse_horizontal_m << " nees_mean=" << nees_mean
            << " gnss_acc=" << result.metrics.gnss_accepted
            << " gnss_gate=" << result.metrics.gnss_gated << " truth_e=" << last.truth_e_m
            << " est_e=" << last.est_e_m << "\n";
  return 0;
}
