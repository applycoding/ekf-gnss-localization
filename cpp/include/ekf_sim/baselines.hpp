// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf_sim/trace.hpp"

namespace ekf::sim {

struct DeadReckoningState {
  double e_m = 0.0;
  double n_m = 0.0;
  double psi_rad = 0.0;
  bool started = false;
};

/** Baseline (a): wheel speed + raw gyro from init-time true pose. */
void AdvanceDeadReckoning(DeadReckoningState& dr, double wheel_speed_m_s, double gyro_z_rad_s,
                          double dt_s);

/** Populate per-tick DR positions into optional parallel arrays (same length as trace). */
void ComputeDeadReckoningTrace(const Trace& sensors, double init_t_s, double init_e, double init_n,
                               double init_psi, std::vector<double>& dr_e,
                               std::vector<double>& dr_n);

}  // namespace ekf::sim
