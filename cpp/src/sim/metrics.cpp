// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/metrics.hpp"

#include "ekf_sim/baselines.hpp"
#include "ekf_sim/constants.hpp"
#include "ekf_sim/scenario.hpp"

#include <cmath>

namespace ekf::sim {

namespace {

double horizontal_error(double e, double n, double te, double tn) {
  return std::hypot(e - te, n - tn);
}

const TickRecord* tick_at_time(const Trace& trace, double t_s) {
  if (trace.ticks.empty()) {
    return nullptr;
  }
  std::size_t idx = static_cast<std::size_t>(t_s / kDtS + 0.5);
  if (idx >= trace.ticks.size()) {
    idx = trace.ticks.size() - 1;
  }
  return &trace.ticks[idx];
}

BaselineMetrics ekf_rmse_after_init(const Trace& trace, double t0_s) {
  BaselineMetrics m{};
  double sum_sq = 0.0;
  std::size_t count = 0;
  for (const TickRecord& rec : trace.ticks) {
    if (!rec.filter_initialized || rec.t_s + 1e-9 < t0_s) {
      continue;
    }
    const double err = horizontal_error(rec.est_e_m, rec.est_n_m, rec.truth_e_m, rec.truth_n_m);
    sum_sq += err * err;
    ++count;
    m.max_horizontal_m = std::max(m.max_horizontal_m, err);
  }
  if (count > 0) {
    m.rmse_horizontal_m = std::sqrt(sum_sq / static_cast<double>(count));
  }
  return m;
}

}  // namespace

RunMetrics ComputeMetrics(const ScenarioSpec& spec, const Trace& trace_on, const Trace& trace_off,
                          double true_gyro_bias_rad_s) {
  RunMetrics m{};
  const double t0 = trace_on.init_time_s > 0.0 ? trace_on.init_time_s : 0.0;
  m.init_time_s = t0;

  m.baseline_ekf_gate_on = ekf_rmse_after_init(trace_on, t0);
  m.baseline_ekf_gate_off = ekf_rmse_after_init(trace_off, t0);
  m.rmse_horizontal_m = m.baseline_ekf_gate_on.rmse_horizontal_m;
  m.max_horizontal_m = m.baseline_ekf_gate_on.max_horizontal_m;

  m.gnss_accepted = trace_on.gnss_accepted;
  m.gnss_gated = trace_on.gnss_gated;
  m.wheel_accepted = trace_on.wheel_accepted;
  m.wheel_gated = trace_on.wheel_gated;
  m.reacquisitions = trace_on.reacquisitions;
  m.final_gyro_bias_rad_s = trace_on.final_b_g_rad_s;
  m.final_wheel_scale = trace_on.final_k;

  double gnss_sum_sq = 0.0;
  std::size_t gnss_epochs = 0;
  std::uint32_t outliers = 0;
  std::uint32_t outliers_rejected = 0;
  std::uint32_t inliers = 0;
  std::uint32_t inliers_rejected = 0;
  std::uint32_t candidates = 0;
  std::uint32_t gated = 0;

  double dropout_sum_sq = 0.0;
  std::size_t dropout_count = 0;
  double first_return_t = -1.0;

  for (const GnssRecord& g : trace_on.gnss) {
    if (g.t_s + 1e-9 < t0) {
      continue;
    }
    if (g.emitted) {
      const double err = horizontal_error(g.meas_e_m, g.meas_n_m, g.truth_e_m, g.truth_n_m);
      gnss_sum_sq += err * err;
      ++gnss_epochs;
      m.baseline_gnss_only.max_horizontal_m = std::max(m.baseline_gnss_only.max_horizontal_m, err);

      if (spec.dropout_start_s >= 0.0 && g.t_s >= spec.dropout_start_s && g.t_s < spec.dropout_end_s) {
        const TickRecord* tk = tick_at_time(trace_on, g.t_s);
        if (tk != nullptr && tk->filter_initialized) {
          const double eerr =
              horizontal_error(tk->est_e_m, tk->est_n_m, tk->truth_e_m, tk->truth_n_m);
          dropout_sum_sq += eerr * eerr;
          ++dropout_count;
          m.dropout_max_m = std::max(m.dropout_max_m, eerr);
        }
      }
      if (first_return_t < 0.0 && spec.dropout_end_s > 0.0 && g.t_s >= spec.dropout_end_s) {
        first_return_t = g.t_s;
      }
    }

    if (g.emitted) {
      ++candidates;
      if (!g.accepted) {
        ++gated;
      }
      if (g.sim_outlier) {
        ++outliers;
        if (!g.accepted) {
          ++outliers_rejected;
        }
      } else {
        ++inliers;
        if (!g.accepted) {
          ++inliers_rejected;
        }
      }
    }
  }

  if (gnss_epochs > 0) {
    m.baseline_gnss_only.rmse_horizontal_m =
        std::sqrt(gnss_sum_sq / static_cast<double>(gnss_epochs));
  }
  if (dropout_count > 0) {
    m.dropout_rmse_m = std::sqrt(dropout_sum_sq / static_cast<double>(dropout_count));
  }

  if (first_return_t > 0.0) {
    const TickRecord* rec = tick_at_time(trace_on, first_return_t + 3.0);
    if (rec != nullptr && rec->filter_initialized) {
      m.error_after_dropout_return_m =
          horizontal_error(rec->est_e_m, rec->est_n_m, rec->truth_e_m, rec->truth_n_m);
    }
  }

  if (candidates > 0) {
    m.gnss_rejection_rate = static_cast<double>(gated) / static_cast<double>(candidates);
  }
  if (outliers > 0) {
    m.outlier_recall = static_cast<double>(outliers_rejected) / static_cast<double>(outliers);
  }
  if (inliers > 0) {
    m.inlier_false_reject_rate =
        static_cast<double>(inliers_rejected) / static_cast<double>(inliers);
  }

  const TickRecord* t90 = tick_at_time(trace_on, 90.0);
  if (t90 != nullptr && t90->filter_initialized) {
    m.ekf_error_at_90s_m =
        horizontal_error(t90->est_e_m, t90->est_n_m, t90->truth_e_m, t90->truth_n_m);
  }

  const TickRecord* t40 = tick_at_time(trace_on, 40.0);
  if (t40 != nullptr && t40->filter_initialized) {
    m.gyro_bias_error_at_40s_rad_s = std::abs(t40->est_b_g_rad_s - true_gyro_bias_rad_s);
  }

  DeadReckoningState dr{};
  const TickRecord* init_rec = tick_at_time(trace_on, t0);
  if (init_rec != nullptr) {
    dr.e_m = init_rec->truth_e_m;
    dr.n_m = init_rec->truth_n_m;
    dr.psi_rad = init_rec->truth_psi_rad;
    dr.started = true;
  }
  double dr_sum_sq = 0.0;
  std::size_t dr_count = 0;
  double last_wheel = 0.0;
  for (const TickRecord& rec : trace_on.ticks) {
    if (rec.t_s + 1e-9 < t0) {
      continue;
    }
    if (rec.have_wheel_meas) {
      last_wheel = rec.meas_wheel_m_s;
    }
    AdvanceDeadReckoning(dr, last_wheel, rec.meas_gyro_z_rad_s, kDtS);
    const double err = horizontal_error(dr.e_m, dr.n_m, rec.truth_e_m, rec.truth_n_m);
    dr_sum_sq += err * err;
    ++dr_count;
    m.baseline_dead_reckoning.max_horizontal_m =
        std::max(m.baseline_dead_reckoning.max_horizontal_m, err);
    if (std::abs(rec.t_s - 90.0) < 0.005) {
      m.dead_reckoning_error_at_90s_m = err;
    }
  }
  if (dr_count > 0) {
    m.baseline_dead_reckoning.rmse_horizontal_m =
        std::sqrt(dr_sum_sq / static_cast<double>(dr_count));
  }

  (void)spec;
  return m;
}

}  // namespace ekf::sim
