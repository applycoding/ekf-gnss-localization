// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace ekf {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);
constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);

constexpr double kGnssGateNis = 9.210340371976184;
constexpr double kWheelGateNis = 6.634896601021214;

constexpr double kNeessPosLower = 1.484439;
constexpr double kNeessPosUpper = 2.591224;
constexpr double kNeessFullLower = 4.161956;
constexpr double kNeessFullUpper = 5.913773;

constexpr double kEnvelopeChi2 = 11.829007;

enum class Status : std::uint8_t {
  Ok,
  Gated,
  Reacquired,
  NotInitialized,
  InvalidDt,
  InvalidInput,
  TimeMismatch,
  Numerical
};

enum class Lifecycle : std::uint8_t {
  NoFix,
  Collecting,
  Initialized
};

struct Llh {
  double lat_rad;
  double lon_rad;
  double h_m;
};

struct Enu {
  double e;
  double n;
  double u;
};

struct Origin {
  Llh llh;
  double ecef[3];
  double R_enu_from_ecef[9];
};

struct ImuSample {
  double t_s;
  double gyro_z_rad_s;
  double accel_x_m_s2;
};

struct WheelSample {
  double t_s;
  double speed_m_s;
};

struct GnssSample {
  double t_s;
  double lat_rad;
  double lon_rad;
  double h_m;
};

struct Config {
  double q_gyro;
  double q_accel;
  double q_bias;
  double q_scale;
  double r_wheel;
  double gnss_sigma_m;
  double gnss_bias_sigma_m;
  double gnss_bias_tau_s;
  bool model_gnss_bias;
  bool estimate_wheel_scale;
  bool gate;
};

struct UpdateResult {
  Status status;
  double nis_pre_gate;
  bool accepted;
};

}  // namespace ekf
