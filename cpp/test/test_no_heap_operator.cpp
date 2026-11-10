// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/types.hpp"

#include <gtest/gtest.h>

TEST(NoHeapOperator, FilterReset) {
  ekf::Filter f{};
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
  c.gate = true;
  EXPECT_EQ(ekf::Reset(f, c, 0.0), ekf::Status::Ok);
}
