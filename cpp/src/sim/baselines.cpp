// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/baselines.hpp"

#include "ekf/geodesy.hpp"
#include "ekf_sim/constants.hpp"

#include <cmath>

namespace ekf::sim {

void AdvanceDeadReckoning(DeadReckoningState& dr, double wheel_speed_m_s, double gyro_z_rad_s,
                          double dt_s) {
  if (!dr.started) {
    return;
  }
  const double psi_mid = dr.psi_rad + 0.5 * gyro_z_rad_s * dt_s;
  dr.e_m += wheel_speed_m_s * std::cos(psi_mid) * dt_s;
  dr.n_m += wheel_speed_m_s * std::sin(psi_mid) * dt_s;
  dr.psi_rad = ekf::WrapPi(dr.psi_rad + gyro_z_rad_s * dt_s);
}

void ComputeDeadReckoningTrace(const Trace& trace, double init_t_s, double init_e, double init_n,
                               double init_psi, std::vector<double>& dr_e,
                               std::vector<double>& dr_n) {
  const std::size_t n = trace.ticks.size();
  dr_e.assign(n, 0.0);
  dr_n.assign(n, 0.0);
  DeadReckoningState dr{};
  dr.e_m = init_e;
  dr.n_m = init_n;
  dr.psi_rad = init_psi;
  dr.started = false;

  for (std::size_t i = 0; i < n; ++i) {
    const double t_s = trace.ticks[i].t_s;
    if (!dr.started) {
      if (t_s + 1e-9 >= init_t_s) {
        dr.started = true;
      }
    }
  if (dr.started) {
      AdvanceDeadReckoning(dr, trace.ticks[i].est_v_m_s, 0.0, kDtS);
    }
    dr_e[i] = dr.e_m;
    dr_n[i] = dr.n_m;
  }
}

}  // namespace ekf::sim
