// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/geodesy.hpp"
#include "ekf/types.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

ekf::Config gated_cfg() {
  ekf::Config c{};
  c.q_gyro = 4e-6;
  c.q_accel = 2.5e-3;
  c.q_bias = 1e-8;
  c.q_scale = 1e-10;
  c.r_wheel = 0.05 * 0.05;
  c.gnss_sigma_m = 1.5;
  c.gnss_bias_sigma_m = 1.0;
  c.gnss_bias_tau_s = 60.0;
  c.model_gnss_bias = false;
  c.estimate_wheel_scale = false;
  c.estimate_gyro_bias = true;
  c.gate = true;
  return c;
}

void init_filter(ekf::Filter& f) {
  f.lifecycle = ekf::Lifecycle::Initialized;
  f.t_s = 0.0;
  f.x[0] = 0.0;
  f.x[1] = 0.0;
  f.x[2] = 0.0;
  f.x[3] = 5.0;
  f.x[4] = 0.0;
  f.x[5] = 1.0;
  for (int i = 0; i < 8; ++i) {
    f.P[i * 8 + i] = 1.0;
  }
  f.P[27] = 0.25;
}

}  // namespace

TEST(Gates, WheelThreshold) {
  ekf::Config cfg = gated_cfg();
  ekf::Filter f{};
  init_filter(f);
  ekf::WheelSample w{0.0, 5.0};
  cfg.gate = false;
  EXPECT_EQ(ekf::UpdateWheel(f, cfg, w).status, ekf::Status::Ok);
  cfg.gate = true;
  ekf::Filter f2{};
  init_filter(f2);
  const double s = f2.P[27] + cfg.r_wheel;
  w.speed_m_s = 5.0 + std::sqrt(ekf::kWheelGateNis * 1.001 * s);
  EXPECT_EQ(ekf::UpdateWheel(f2, cfg, w).status, ekf::Status::Gated);
}

TEST(Gates, GnssThreshold) {
  const ekf::Config cfg = gated_cfg();
  ekf::Origin origin{};
  ASSERT_EQ(ekf::MakeOrigin({0.0, 0.0, 0.0}, origin), ekf::Status::Ok);
  ekf::Filter f{};
  init_filter(f);
  double ecef[3];
  const ekf::Enu enu_ok{std::sqrt(ekf::kGnssGateNis * 0.999 * (f.P[0] + cfg.gnss_sigma_m * cfg.gnss_sigma_m)),
                        0.0, 0.0};
  ekf::EnuToEcef(origin, enu_ok, ecef);
  ekf::Llh llh{};
  ASSERT_EQ(ekf::EcefToGeodetic(ecef, llh), ekf::Status::Ok);
  ekf::GnssSample gnss{0.0, llh.lat_rad, llh.lon_rad, llh.h_m};
  EXPECT_NE(ekf::UpdateGnss(f, cfg, origin, gnss).status, ekf::Status::Gated);

  ekf::Filter f2{};
  init_filter(f2);
  const ekf::Enu enu_bad{std::sqrt(ekf::kGnssGateNis * 1.001 * (f2.P[0] + cfg.gnss_sigma_m * cfg.gnss_sigma_m)),
                         0.0, 0.0};
  ekf::EnuToEcef(origin, enu_bad, ecef);
  ASSERT_EQ(ekf::EcefToGeodetic(ecef, llh), ekf::Status::Ok);
  gnss = {0.0, llh.lat_rad, llh.lon_rad, llh.h_m};
  EXPECT_EQ(ekf::UpdateGnss(f2, cfg, origin, gnss).status, ekf::Status::Gated);
}
