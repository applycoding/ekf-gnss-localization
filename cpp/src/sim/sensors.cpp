// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/sensors.hpp"

#include <cmath>

namespace ekf::sim {

SensorSimulator::SensorSimulator(const ScenarioSpec& spec, Rng& rng, const ekf::Origin& origin)
    : spec_(spec), rng_(rng), origin_(origin) {
  state_.gyro_bias_rad_s = spec.gyro_bias_rad_s;
}

void SensorSimulator::evolve_gnss_bias(double dt_s) {
  if (!spec_.gnss_gauss_markov) {
    state_.beta_e_m = 0.0;
    state_.beta_n_m = 0.0;
    return;
  }
  if (!state_.beta_initialized) {
    state_.beta_e_m = rng_.normal(RngStream::GnssGm) * kGnssBiasSigmaM;
    state_.beta_n_m = rng_.normal(RngStream::GnssGm) * kGnssBiasSigmaM;
    state_.beta_initialized = true;
  }
  const double phi = std::exp(-dt_s / kGnssBiasTauS);
  const double drive = std::sqrt(1.0 - phi * phi);
  state_.beta_e_m = phi * state_.beta_e_m + kGnssBiasSigmaM * drive * rng_.normal(RngStream::GnssGm);
  state_.beta_n_m = phi * state_.beta_n_m + kGnssBiasSigmaM * drive * rng_.normal(RngStream::GnssGm);
}

bool SensorSimulator::in_dropout(double t_s) const {
  if (spec_.dropout_start_s < 0.0) {
    return false;
  }
  return t_s >= spec_.dropout_start_s && t_s < spec_.dropout_end_s;
}

bool SensorSimulator::in_slip(double t_s) const {
  for (int i = 0; i < spec_.slip_window_count; ++i) {
    const SlipWindow& w = spec_.slip_windows[i];
    if (t_s >= w.start_s && t_s < w.end_s) {
      return true;
    }
  }
  return false;
}

ImuTruth SensorSimulator::imu_at_tick(int tick, const TruthState& /*truth*/) {
  const double t_s = tick * kDtS;
  ImuTruth out{};
  out.omega_true_rad_s = yaw_rate_omega(t_s);
  out.accel_true_m_s2 = speed_derivative(t_s);
  out.sample.t_s = t_s;
  out.sample.gyro_z_rad_s =
      out.omega_true_rad_s + state_.gyro_bias_rad_s + kGyroMeasSigmaRadS * rng_.normal(RngStream::Gyro);
  out.sample.accel_x_m_s2 = out.accel_true_m_s2 + kAccelMeasSigmaMS2 * rng_.normal(RngStream::Accel);
  return out;
}

ekf::WheelSample SensorSimulator::wheel_at_tick(int tick, const TruthState& truth) {
  const double t_s = tick * kDtS;
  ekf::WheelSample w{};
  w.t_s = t_s;
  double z = spec_.wheel_scale_true * truth.v_m_s +
             kWheelMeasSigmaMS * rng_.normal(RngStream::Wheel);
  if (in_slip(t_s)) {
    z *= 0.5;
  }
  w.speed_m_s = z;
  return w;
}

ekf::GnssSample SensorSimulator::meas_enu_to_gnss(double t_s, double meas_e, double meas_n) const {
  ekf::GnssSample gnss{};
  gnss.t_s = t_s;
  const ekf::Enu enu{meas_e, meas_n, 0.0};
  double ecef[3];
  ekf::EnuToEcef(origin_, enu, ecef);
  ekf::Llh llh{};
  if (ekf::EcefToGeodetic(ecef, llh) != ekf::Status::Ok) {
    gnss.lat_rad = kOriginLatRad;
    gnss.lon_rad = kOriginLonRad;
    gnss.h_m = kOriginHeightM;
    return gnss;
  }
  gnss.lat_rad = llh.lat_rad;
  gnss.lon_rad = llh.lon_rad;
  gnss.h_m = llh.h_m;
  return gnss;
}

GnssTruthLabel SensorSimulator::gnss_at_tick(int tick, const TruthState& truth, ekf::GnssSample& out) {
  const double t_s = tick * kDtS;
  GnssTruthLabel label{};
  if (in_dropout(t_s)) {
    label.emitted = false;
    return label;
  }
  label.emitted = true;
  double meas_e = truth.pe_m + state_.beta_e_m + kGnssWhiteSigmaM * rng_.normal(RngStream::GnssWhiteE);
  double meas_n = truth.pn_m + state_.beta_n_m + kGnssWhiteSigmaM * rng_.normal(RngStream::GnssWhiteN);

  if (spec_.multipath_probability > 0.0 &&
      rng_.bernoulli(RngStream::OutlierDraw, spec_.multipath_probability)) {
    label.is_outlier = true;
    const double ang = rng_.uniform(RngStream::OutlierAngle, 0.0, 2.0 * ekf::kPi);
    const double len = rng_.uniform(RngStream::OutlierLength, kMultipathMinM, kMultipathMaxM);
    meas_e += len * std::cos(ang);
    meas_n += len * std::sin(ang);
  }

  label.meas_e_m = meas_e;
  label.meas_n_m = meas_n;
  out = meas_enu_to_gnss(t_s, meas_e, meas_n);
  return label;
}

}  // namespace ekf::sim
