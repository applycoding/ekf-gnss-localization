// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace ekf::sim {

struct TruthState {
  double pe_m = 0.0;
  double pn_m = 0.0;
  double psi_rad = 0.0;
  double v_m_s = 0.0;
};

/** Piecewise speed v(t) from PLAN.md (m/s). */
double speed_v(double t_s);

/** Piecewise yaw rate ω(t) from PLAN.md (rad/s). */
double yaw_rate_omega(double t_s);

/** Analytic dv/dt for accelerometer truth (m/s²). */
double speed_derivative(double t_s);

/** Midpoint integrator at 100 Hz; advances state from tick time t to t + dt. */
void integrate_truth_midpoint(TruthState& state, double t_s, double dt_s);

}  // namespace ekf::sim
