// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/geodesy.hpp"
#include "ekf/types.hpp"
#include "ekf_sim/constants.hpp"
#include "ekf_sim/rng.hpp"
#include "ekf_sim/scenario.hpp"
#include "ekf_sim/trajectory.hpp"

namespace ekf::sim {

struct GnssTruthLabel {
  bool emitted = false;
  bool is_outlier = false;
  double meas_e_m = 0.0;
  double meas_n_m = 0.0;
};

struct SensorState {
  double beta_e_m = 0.0;
  double beta_n_m = 0.0;
  double gyro_bias_rad_s = kDefaultGyroBiasRadS;
  bool beta_initialized = false;
};

struct ImuTruth {
  ekf::ImuSample sample{};
  double omega_true_rad_s = 0.0;
  double accel_true_m_s2 = 0.0;
};

class SensorSimulator {
 public:
  SensorSimulator(const ScenarioSpec& spec, Rng& rng, const ekf::Origin& origin);

  void evolve_gnss_bias(double dt_s);
  ImuTruth imu_at_tick(int tick, const TruthState& truth);
  ekf::WheelSample wheel_at_tick(int tick, const TruthState& truth);
  GnssTruthLabel gnss_at_tick(int tick, const TruthState& truth, ekf::GnssSample& out);

 private:
  const ScenarioSpec& spec_;
  Rng& rng_;
  const ekf::Origin& origin_;
  SensorState state_;

  bool in_dropout(double t_s) const;
  bool in_slip(double t_s) const;
  ekf::GnssSample meas_enu_to_gnss(double t_s, double meas_e, double meas_n) const;
};

}  // namespace ekf::sim
