// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace ekf {

/** Planar 2-state covariance in row-major [P00,P01,P10,P11]. */
struct Linear2State {
  double x[2];
  double P[4];
};

void Linear2Predict(Linear2State& s, const double F[4], const double Q[4]);

/** Joseph GNSS-style update; H is measurement row [h0,h1], returns NIS. */
double Linear2JosephUpdate(Linear2State& s, double z, double R, const double H[2]);

}  // namespace ekf
