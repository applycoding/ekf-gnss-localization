// SPDX-License-Identifier: Apache-2.0
#include "ekf/linear2.hpp"

#include <cmath>
#include <cstring>

namespace ekf {

void Linear2Predict(Linear2State& s, const double F[4], const double Q[4]) {
  const double x0 = s.x[0];
  const double x1 = s.x[1];
  s.x[0] = F[0] * x0 + F[1] * x1;
  s.x[1] = F[2] * x0 + F[3] * x1;
  double FP[4];
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      FP[i * 2 + j] = F[i * 2 + 0] * s.P[0 * 2 + j] + F[i * 2 + 1] * s.P[1 * 2 + j];
    }
  }
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      s.P[i * 2 + j] = FP[i * 2 + 0] * F[j * 2 + 0] + FP[i * 2 + 1] * F[j * 2 + 1];
    }
  }
  for (int i = 0; i < 4; ++i) {
    s.P[i] += Q[i];
  }
}

double Linear2JosephUpdate(Linear2State& s, double z, double R, const double H[2]) {
  const Linear2State backup = s;
  const double innov = z - (H[0] * s.x[0] + H[1] * s.x[1]);
  const double S = H[0] * (s.P[0] * H[0] + s.P[1] * H[1]) + H[1] * (s.P[2] * H[0] + s.P[3] * H[1]) + R;
  double nis = innov * innov / S;
  double K[2];
  K[0] = (s.P[0] * H[0] + s.P[1] * H[1]) / S;
  K[1] = (s.P[2] * H[0] + s.P[3] * H[1]) / S;
  s.x[0] += K[0] * innov;
  s.x[1] += K[1] * innov;

  double IKH[4];
  IKH[0] = 1.0 - K[0] * H[0];
  IKH[1] = -K[0] * H[1];
  IKH[2] = -K[1] * H[0];
  IKH[3] = 1.0 - K[1] * H[1];
  double Pnew[4];
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 2; ++k) {
        double mid = 0.0;
        for (int l = 0; l < 2; ++l) {
          mid += IKH[i * 2 + l] * backup.P[l * 2 + k];
        }
        sum += mid * IKH[j * 2 + k];
      }
      Pnew[i * 2 + j] = sum + K[i] * R * K[j];
    }
  }
  std::memcpy(s.P, Pnew, sizeof(Pnew));
  return nis;
}

}  // namespace ekf
