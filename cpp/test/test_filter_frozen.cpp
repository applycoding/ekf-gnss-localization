// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/geodesy.hpp"
#include "ekf/mat.hpp"

#include <cstring>

#include <gtest/gtest.h>

namespace {

ekf::Config frozen_cfg() {
  ekf::Config c{};
  c.q_gyro = 4e-6;
  c.q_accel = 2.5e-3;
  c.q_bias = 1e-8;
  c.q_scale = 1e-10;
  c.r_wheel = 0.05 * 0.05;
  c.gnss_sigma_m = 1.5;
  c.gnss_bias_sigma_m = 1.0;
  c.gnss_bias_tau_s = 60.0;
  c.model_gnss_bias = true;
  c.estimate_wheel_scale = true;
  c.gate = false;
  return c;
}

void set_frozen_state(ekf::Filter& f) {
  f.lifecycle = ekf::Lifecycle::Initialized;
  f.t_s = 0.0;
  f.x[0] = 10.0;
  f.x[1] = -5.0;
  f.x[2] = 0.5;
  f.x[3] = 8.0;
  f.x[4] = 0.01;
  f.x[5] = 1.0;
  f.x[6] = 0.3;
  f.x[7] = -0.2;
  std::memset(f.P, 0, sizeof(f.P));
  f.P[0] = 2.0;
  f.P[9] = 2.0;
  f.P[18] = 0.04;
  f.P[27] = 0.25;
  f.P[36] = 4e-4;
  f.P[45] = 9e-4;
  f.P[54] = 1.0;
  f.P[63] = 1.0;
  f.P[6] = -1.0;
  f.P[15] = -1.0;
  f.P[48] = -1.0;
  f.P[57] = -1.0;
}

}  // namespace

TEST(FilterFrozen, PredictWheelGnss) {
  ekf::Filter f{};
  set_frozen_state(f);
  const ekf::Config cfg = frozen_cfg();

  ekf::ImuSample imu{0.02, 0.31, 0.2};
  ASSERT_EQ(ekf::Predict(f, cfg, imu, 0.02), ekf::Status::Ok);
  EXPECT_NEAR(f.x[0], 10.140413209900, 1e-9);
  EXPECT_NEAR(f.x[1], -4.923291913823, 1e-9);
  EXPECT_NEAR(f.x[2], 0.506, 1e-9);
  EXPECT_NEAR(f.x[3], 8.004, 1e-9);
  EXPECT_NEAR(f.P[0], 2.000312380335, 1e-9);
  EXPECT_NEAR(f.P[9], 2.000811619665, 1e-9);

  ekf::WheelSample wheel{0.02, 8.1};
  const ekf::UpdateResult wres = ekf::UpdateWheel(f, cfg, wheel);
  ASSERT_TRUE(wres.accepted);
  EXPECT_NEAR(f.x[3], 8.081383013426, 1e-9);
  EXPECT_NEAR(f.x[5], 1.002229299247, 1e-9);
  EXPECT_NEAR(wres.nis_pre_gate, 0.029709135329, 1e-9);

  ekf::Origin origin{};
  const ekf::Llh origin_llh{0.0, 0.0, 0.0};
  ASSERT_EQ(ekf::MakeOrigin(origin_llh, origin), ekf::Status::Ok);
  double ecef_z[3];
  const ekf::Enu z_enu{12.0, -3.5, 0.0};
  ekf::EnuToEcef(origin, z_enu, ecef_z);
  ekf::Llh gnss_llh{};
  ASSERT_EQ(ekf::EcefToGeodetic(ecef_z, gnss_llh), ekf::Status::Ok);
  const ekf::GnssSample gnss{0.02, gnss_llh.lat_rad, gnss_llh.lon_rad, gnss_llh.h_m};
  const ekf::UpdateResult gres = ekf::UpdateGnss(f, cfg, origin, gnss);
  ASSERT_EQ(gres.status, ekf::Status::Ok);
  ASSERT_TRUE(gres.accepted);
  EXPECT_NEAR(f.x[0], 10.621255435500, 1e-9);
  EXPECT_NEAR(f.x[1], -4.423126700443, 1e-9);
  EXPECT_NEAR(f.x[2], 0.507332004748, 1e-9);
  EXPECT_NEAR(gres.nis_pre_gate, 1.556809790907, 1e-9);
  EXPECT_NEAR(f.P[0], 1.692285622328, 1e-9);
}
