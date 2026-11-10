// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/trajectory.hpp"

#include "ekf/geodesy.hpp"

#include <cmath>

namespace ekf::sim {

double speed_v(double t_s) {
  if (t_s >= 125.0) {
    return 0.0;
  }
  if (t_s < 15.0) {
    return 8.0 * t_s / 15.0;
  }
  if (t_s < 115.0) {
    return 8.0;
  }
  return 8.0 - 0.8 * (t_s - 115.0);
}

double yaw_rate_omega(double t_s) {
  if (t_s >= 40.0 && t_s < 70.0) {
    return 8.0 / 25.0;
  }
  if (t_s >= 85.0 && t_s < 100.0) {
    return 0.4;
  }
  if (t_s >= 100.0 && t_s < 115.0) {
    return -0.4;
  }
  return 0.0;
}

double speed_derivative(double t_s) {
  if (t_s >= 125.0) {
    return 0.0;
  }
  if (t_s < 15.0) {
    return 8.0 / 15.0;
  }
  if (t_s < 115.0) {
    return 0.0;
  }
  return -0.8;
}

void integrate_truth_midpoint(TruthState& state, double t_s, double dt_s) {
  const double t_mid = t_s + 0.5 * dt_s;
  const double omega = yaw_rate_omega(t_mid);
  const double v_mid = speed_v(t_mid);
  const double psi_mid = state.psi_rad + 0.5 * omega * dt_s;
  state.pe_m += v_mid * std::cos(psi_mid) * dt_s;
  state.pn_m += v_mid * std::sin(psi_mid) * dt_s;
  state.psi_rad = ekf::WrapPi(state.psi_rad + omega * dt_s);
  state.v_m_s = speed_v(t_s + dt_s);
}

}  // namespace ekf::sim
