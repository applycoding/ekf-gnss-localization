// SPDX-License-Identifier: Apache-2.0
#include "ekf/linear2.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

double min_eig2(const double P[4]) {
  const double tr = P[0] + P[3];
  const double det = P[0] * P[3] - P[1] * P[2];
  const double disc = std::sqrt(std::max(0.0, tr * tr - 4.0 * det));
  return 0.5 * (tr - disc);
}

}  // namespace

TEST(Exact2State, PredictUpdateJoseph) {
  const double F[4] = {1.0, 1.0, 0.0, 1.0};
  const double Q[4] = {0.0, 0.0, 0.0, 0.0};
  ekf::Linear2State s{};
  s.x[0] = 0.0;
  s.x[1] = 1.0;
  s.P[0] = 1.0;
  s.P[1] = 0.0;
  s.P[2] = 0.0;
  s.P[3] = 1.0;
  ekf::Linear2Predict(s, F, Q);
  EXPECT_NEAR(s.x[0], 1.0, 1e-12);
  EXPECT_NEAR(s.x[1], 1.0, 1e-12);
  EXPECT_NEAR(s.P[0], 2.0, 1e-12);
  EXPECT_NEAR(s.P[1], 1.0, 1e-12);
  EXPECT_NEAR(s.P[2], 1.0, 1e-12);
  EXPECT_NEAR(s.P[3], 1.0, 1e-12);

  const double H[2] = {1.0, 0.0};
  const double nis = ekf::Linear2JosephUpdate(s, 2.0, 1.0, H);
  EXPECT_NEAR(s.x[0], 5.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.x[1], 4.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.P[0], 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.P[1], 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.P[2], 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(s.P[3], 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(nis, 1.0 / 3.0, 1e-12);
  EXPECT_GT(min_eig2(s.P), 0.0);
}
