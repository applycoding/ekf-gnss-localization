// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf_sim/trace.hpp"

#include <cstdint>

namespace ekf::sim {

struct ScenarioSpec;

struct BaselineMetrics {
  double rmse_horizontal_m = 0.0;
  double max_horizontal_m = 0.0;
};

struct RunMetrics {
  double init_time_s = -1.0;
  double rmse_horizontal_m = 0.0;
  double max_horizontal_m = 0.0;
  double dropout_rmse_m = 0.0;
  double dropout_max_m = 0.0;
  double error_after_dropout_return_m = 0.0;
  double dead_reckoning_error_at_90s_m = 0.0;
  double ekf_error_at_90s_m = 0.0;
  double gyro_bias_error_at_40s_rad_s = 0.0;
  double final_wheel_scale = 1.0;
  double final_gyro_bias_rad_s = 0.0;
  std::uint32_t gnss_accepted = 0;
  std::uint32_t gnss_gated = 0;
  std::uint32_t wheel_accepted = 0;
  std::uint32_t wheel_gated = 0;
  std::uint32_t reacquisitions = 0;
  double gnss_rejection_rate = 0.0;
  double outlier_recall = 0.0;
  double inlier_false_reject_rate = 0.0;
  BaselineMetrics baseline_dead_reckoning{};
  BaselineMetrics baseline_gnss_only{};
  BaselineMetrics baseline_ekf_gate_off{};
  BaselineMetrics baseline_ekf_gate_on{};
};

RunMetrics ComputeMetrics(const ScenarioSpec& spec, const Trace& trace_on,
                          const Trace& trace_off, double true_gyro_bias_rad_s);

}  // namespace ekf::sim
