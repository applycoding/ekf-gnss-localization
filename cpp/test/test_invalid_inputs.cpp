// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

namespace {

ekf::Config base_cfg() {
  ekf::Config c{};
  c.q_gyro = 4e-6;
  c.q_accel = 2.5e-3;
  c.q_bias = 1e-8;
  c.q_scale = 1e-10;
  c.r_wheel = 0.0025;
  c.gnss_sigma_m = 1.5;
  c.gnss_bias_sigma_m = 1.0;
  c.gnss_bias_tau_s = 60.0;
  c.model_gnss_bias = true;
  c.estimate_wheel_scale = true;
  c.estimate_gyro_bias = true;
  c.gate = true;
  return c;
}

void set_initialized(ekf::Filter& f) {
  f.lifecycle = ekf::Lifecycle::Initialized;
  f.t_s = 1.0;
  f.x[0] = 1.0;
  f.x[2] = 0.1;
  f.x[3] = 5.0;
  for (int i = 0; i < 8; ++i) {
    f.P[i * 8 + i] = 1.0;
  }
}

void expect_unchanged(const ekf::Filter& before, const ekf::Filter& after) {
  EXPECT_EQ(0, std::memcmp(&before, &after, sizeof(ekf::Filter)));
}

}  // namespace

TEST(InvalidInputs, PredictGuards) {
  ekf::Filter f{};
  const ekf::Config cfg = base_cfg();
  set_initialized(f);
  const ekf::Filter backup = f;
  ekf::ImuSample imu{1.0, 0.0, 0.0};
  EXPECT_EQ(ekf::Predict(f, cfg, imu, 0.0), ekf::Status::InvalidDt);
  expect_unchanged(backup, f);
  EXPECT_EQ(ekf::Predict(f, cfg, imu, 0.03), ekf::Status::InvalidDt);
  expect_unchanged(backup, f);
  imu.gyro_z_rad_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ekf::Predict(f, cfg, imu, 0.01), ekf::Status::InvalidInput);
  expect_unchanged(backup, f);
  imu.gyro_z_rad_s = 0.0;
  imu.t_s = 5.0;
  EXPECT_EQ(ekf::Predict(f, cfg, imu, 0.01), ekf::Status::TimeMismatch);
  expect_unchanged(backup, f);
}

TEST(InvalidInputs, NumericalNonPdP) {
  ekf::Filter f{};
  const ekf::Config cfg = base_cfg();
  set_initialized(f);
  f.x[5] = 1.0;
  for (int i = 0; i < 8; ++i) {
    f.P[i * 8 + i] = 1.0;
  }
  f.P[0] = -1.0;
  const ekf::Filter backup = f;
  ekf::WheelSample wheel{1.0, 5.0};
  EXPECT_EQ(ekf::UpdateWheel(f, cfg, wheel).status, ekf::Status::Numerical);
  expect_unchanged(backup, f);
}
