// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {

void emit_parity_full(const ekf::sim::RunResult& result) {
  std::cout << std::setprecision(17);
  std::cout << "RMSE " << result.metrics.rmse_horizontal_m << "\n";
  std::cout << "GNSS_ACCEPTED " << result.trace.gnss_accepted << "\n";
  std::cout << "GNSS_GATED " << result.trace.gnss_gated << "\n";
  for (const auto& g : result.trace.gnss) {
    if (!g.emitted) {
      continue;
    }
    std::cout << "G " << g.t_s << " " << (g.accepted ? 1 : 0) << " "
              << static_cast<unsigned>(g.update_status) << "\n";
  }
  for (const auto& t : result.trace.ticks) {
    if (!t.filter_initialized) {
      continue;
    }
    std::cout << "S " << t.t_s << " " << t.est_e_m << " " << t.est_n_m << " " << t.est_psi_rad
              << " " << t.est_v_m_s << " " << t.est_b_g_rad_s << " " << t.est_k << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  const char* scenario = "matched";
  std::uint64_t seed = 1;
  bool parity_full = false;
  if (argc > 1) {
    scenario = argv[1];
  }
  if (argc > 2) {
    if (std::strcmp(argv[2], "--parity") == 0 || std::strcmp(argv[2], "--parity-full") == 0) {
      parity_full = true;
    } else {
      seed = static_cast<std::uint64_t>(std::strtoull(argv[2], nullptr, 10));
    }
  }
  if (argc > 3) {
    if (std::strcmp(argv[3], "--parity") == 0 || std::strcmp(argv[3], "--parity-full") == 0) {
      parity_full = true;
    }
  }
  const ekf::sim::RunResult result = ekf::sim::RunScenario(scenario, seed);
  if (parity_full) {
    std::cout << "SCENARIO " << scenario << " SEED " << seed << "\n";
    emit_parity_full(result);
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
