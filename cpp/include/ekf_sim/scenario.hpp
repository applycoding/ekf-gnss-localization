// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/types.hpp"
#include "ekf_sim/constants.hpp"
#include "ekf_sim/metrics.hpp"
#include "ekf_sim/trace.hpp"

#include <cstdint>
#include <string>

namespace ekf::sim {

struct SlipWindow {
  double start_s = 0.0;
  double end_s = 0.0;
};

struct ScenarioSpec {
  std::string name;
  ekf::Config filter{};
  double wheel_scale_true = 1.0;
  double gyro_bias_rad_s = kDefaultGyroBiasRadS;
  bool gnss_gauss_markov = false;
  double dropout_start_s = -1.0;
  double dropout_end_s = -1.0;
  double multipath_probability = 0.0;
  SlipWindow slip_windows[2]{};
  int slip_window_count = 0;
};

/** Load scenario JSON from a file path (repo scenarios/). */
ScenarioSpec LoadScenarioFile(const char* path);

/** Builtin defaults if path is null or matches known scenario names. */
ScenarioSpec ScenarioMatched();
ScenarioSpec ScenarioUrban();
ScenarioSpec ScenarioUrbanWhiteModel();

ScenarioSpec LoadScenarioByName(const char* name);

struct RunResult {
  RunMetrics metrics{};
  Trace trace{};
};

RunResult RunScenario(const ScenarioSpec& spec, std::uint64_t seed);
RunResult RunScenario(const char* scenario_name, std::uint64_t seed);

}  // namespace ekf::sim
