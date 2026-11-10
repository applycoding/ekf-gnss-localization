// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/rng.hpp"
#include "ekf_sim/trajectory.hpp"

#include <gtest/gtest.h>

TEST(Rng, SplitmixStreamDeterministic) {
  ekf::sim::Rng a(42);
  ekf::sim::Rng b(42);
  for (int i = 0; i < 20; ++i) {
    EXPECT_DOUBLE_EQ(a.normal(ekf::sim::RngStream::Gyro), b.normal(ekf::sim::RngStream::Gyro));
  }
}

TEST(Trajectory, SpeedEndpoints) {
  EXPECT_NEAR(ekf::sim::speed_v(0.0), 0.0, 1e-12);
  EXPECT_NEAR(ekf::sim::speed_v(15.0), 8.0, 1e-12);
  EXPECT_NEAR(ekf::sim::speed_v(125.0), 0.0, 1e-12);
}
