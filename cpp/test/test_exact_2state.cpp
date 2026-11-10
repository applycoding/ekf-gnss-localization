// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include <gtest/gtest.h>

namespace {

// PLAN 2-state example: x=[0,1], P=I, F=[[1,1],[0,1]], Q=0.
void mat2_predict(const double x[2], const double P[4], const double F[4], double x_out[2],
                  double P_out[4]) {
  x_out[0] = F[0] * x[0] + F[1] * x[1];
  x_out[1] = F[2] * x[0] + F[3] * x[1];
  const double Ft[4] = {F[0], F[2], F[1], F[3]};
  double FP[4];
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      FP[i * 2 + j] = F[i * 2 + 0] * P[0 * 2 + j] + F[i * 2 + 1] * P[1 * 2 + j];
    }
  }
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      P_out[i * 2 + j] = FP[i * 2 + 0] * Ft[0 * 2 + j] + FP[i * 2 + 1] * Ft[1 * 2 + j];
    }
  }
}

void joseph_update(const double x[2], const double P[4], double z, double R, const double H[2],
                   double x_out[2], double P_out[4], double& nis) {
  const double innov = z - (H[0] * x[0] + H[1] * x[1]);
  const double S = H[0] * (P[0] * H[0] + P[1] * H[1]) + H[1] * (P[2] * H[0] + P[3] * H[1]) + R;
  nis = innov * innov / S;
  const double K0 = (P[0] * H[0] + P[1] * H[1]) / S;
  const double K1 = (P[2] * H[0] + P[3] * H[1]) / S;
  x_out[0] = x[0] + K0 * innov;
  x_out[1] = x[1] + K1 * innov;
  const double IKH[4] = {1.0 - K0 * H[0], -K0 * H[1], -K1 * H[0], 1.0 - K1 * H[1]};
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 2; ++k) {
        double mid = 0.0;
        for (int l = 0; l < 2; ++l) {
          mid += IKH[i * 2 + l] * P[l * 2 + k];
        }
        sum += mid * IKH[j * 2 + k];
      }
      const double karr[2] = {K0, K1};
      P_out[i * 2 + j] = sum + karr[i] * R * karr[j];
    }
  }
}

double min_eig2(const double P[4]) {
  const double tr = P[0] + P[3];
  const double det = P[0] * P[3] - P[1] * P[2];
  const double disc = std::sqrt(std::max(0.0, tr * tr - 4.0 * det));
  return 0.5 * (tr - disc);
}

}  // namespace

TEST(Exact2State, PredictUpdateJoseph) {
  const double F[4] = {1.0, 1.0, 0.0, 1.0};
  const double x0[2] = {0.0, 1.0};
  const double P0[4] = {1.0, 0.0, 0.0, 1.0};
  double x1[2];
  double P1[4];
  mat2_predict(x0, P0, F, x1, P1);
  EXPECT_NEAR(x1[0], 1.0, 1e-12);
  EXPECT_NEAR(x1[1], 1.0, 1e-12);
  EXPECT_NEAR(P1[0], 2.0, 1e-12);
  EXPECT_NEAR(P1[1], 1.0, 1e-12);
  EXPECT_NEAR(P1[2], 1.0, 1e-12);
  EXPECT_NEAR(P1[3], 1.0, 1e-12);

  const double H[2] = {1.0, 0.0};
  double x2[2];
  double P2[4];
  double nis = 0.0;
  joseph_update(x1, P1, 2.0, 1.0, H, x2, P2, nis);
  EXPECT_NEAR(x2[0], 5.0 / 3.0, 1e-12);
  EXPECT_NEAR(x2[1], 4.0 / 3.0, 1e-12);
  EXPECT_NEAR(P2[0], 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(P2[1], 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(P2[2], 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(P2[3], 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(nis, 1.0 / 3.0, 1e-12);
  EXPECT_GT(min_eig2(P2), 0.0);
}
