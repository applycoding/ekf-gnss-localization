// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

namespace ekf::sim {

struct TickRecord {
  double t_s = 0.0;
  double truth_e_m = 0.0;
  double truth_n_m = 0.0;
  double truth_psi_rad = 0.0;
  double est_e_m = 0.0;
  double est_n_m = 0.0;
  double est_psi_rad = 0.0;
  double est_v_m_s = 0.0;
  bool filter_initialized = false;
  double p_pe = 0.0;
  double p_pn = 0.0;
  double p_psi = 0.0;
  double envelope_m = 0.0;
  double meas_gyro_z_rad_s = 0.0;
  double meas_wheel_m_s = 0.0;
  bool have_wheel_meas = false;
  double est_b_g_rad_s = 0.0;
  double est_k = 1.0;
};

struct GnssRecord {
  double t_s = 0.0;
  double meas_e_m = 0.0;
  double meas_n_m = 0.0;
  double truth_e_m = 0.0;
  double truth_n_m = 0.0;
  bool emitted = false;
  bool sim_outlier = false;
  bool accepted = false;
  double nis_pre_gate = 0.0;
  double nees_pos = 0.0;
  double nees_full = 0.0;
  std::uint8_t update_status = 0;
};

struct WheelRecord {
  double t_s = 0.0;
  double nis_pre_gate = 0.0;
  bool accepted = false;
};

struct Trace {
  std::vector<TickRecord> ticks;
  std::vector<GnssRecord> gnss;
  std::vector<WheelRecord> wheel;
  double init_time_s = -1.0;
  std::uint64_t seed = 0;
  char scenario_name[64]{};
  std::uint32_t gnss_accepted = 0;
  std::uint32_t gnss_gated = 0;
  std::uint32_t wheel_accepted = 0;
  std::uint32_t wheel_gated = 0;
  std::uint32_t reacquisitions = 0;
  double final_b_g_rad_s = 0.0;
  double final_k = 1.0;
};

void TraceClear(Trace& trace);
void TraceReserve(Trace& trace);

}  // namespace ekf::sim
