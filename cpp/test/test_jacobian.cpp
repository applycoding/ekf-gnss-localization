// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/mat.hpp"

#include <cmath>
#include <cstring>

#include <gtest/gtest.h>

namespace {

ekf::Filter make_state(double pe, double pn, double psi, double v, double bg, double k, double b0,
                       double b1) {
  ekf::Filter f{};
  f.lifecycle = ekf::Lifecycle::Initialized;
  f.t_s = 1.0;
  f.x[0] = pe;
  f.x[1] = pn;
  f.x[2] = psi;
  f.x[3] = v;
  f.x[4] = bg;
  f.x[5] = k;
  f.x[6] = b0;
  f.x[7] = b1;
  for (int i = 0; i < 8; ++i) {
    f.P[i * 8 + i] = 1.0;
  }
  return f;
}

bool predict_x(const ekf::Filter& in, const ekf::Config& cfg, const ekf::ImuSample& imu, double dt,
             double x_out[8]) {
  ekf::Filter f = in;
  if (ekf::Predict(f, cfg, imu, dt) != ekf::Status::Ok) {
    return false;
  }
  std::memcpy(x_out, f.x, sizeof(f.x));
  return true;
}

double diff_angle(double a, double b) {
  return ekf::WrapPi(a - b);
}

}  // namespace

TEST(Jacobian, CentralDifferenceFiftyStates) {
  ekf::Config cfg{};
  cfg.q_gyro = 4e-6;
  cfg.q_accel = 2.5e-3;
  cfg.q_bias = 1e-8;
  cfg.q_scale = 1e-10;
  cfg.r_wheel = 0.05 * 0.05;
  cfg.gnss_sigma_m = 1.5;
  cfg.gnss_bias_sigma_m = 1.0;
  cfg.gnss_bias_tau_s = 60.0;
  cfg.model_gnss_bias = true;
  cfg.estimate_wheel_scale = true;
  cfg.estimate_gyro_bias = true;
  cfg.gate = false;

  const double dt = 0.02;
  const double h = 1e-6;
  const ekf::ImuSample imu{1.02, 0.2, 0.1};

  for (int n = 0; n < 50; ++n) {
    const double u = static_cast<double>(n);
    const double psi = ekf::kPi * (0.95 - 0.04 * u);
    const double v = (n % 5 == 0) ? 0.0 : (n % 7 == 0 ? -2.0 : 5.0 + 0.1 * u);
    ekf::Filter base = make_state(10.0 + u, -5.0, psi, v, 0.01, 1.0, 0.1, -0.05);
    double F[64];
    ekf::PredictJacobian(base, cfg, dt, F);

    double x_nom[8];
    ASSERT_TRUE(predict_x(base, cfg, imu, dt, x_nom));

    for (int j = 0; j < 8; ++j) {
      ekf::Filter fp = base;
      ekf::Filter fm = base;
      fp.x[j] += h;
      fm.x[j] -= h;
      double xp[8];
      double xm[8];
      ASSERT_TRUE(predict_x(fp, cfg, imu, dt, xp));
      ASSERT_TRUE(predict_x(fm, cfg, imu, dt, xm));
      for (int i = 0; i < 8; ++i) {
        const double fd =
            (i == 2) ? diff_angle(xp[i], xm[i]) / (2.0 * h) : (xp[i] - xm[i]) / (2.0 * h);
        const double tol = 1e-7 + 1e-6 * std::abs(F[i * 8 + j]);
        EXPECT_NEAR(F[i * 8 + j], fd, tol) << "n=" << n << " i=" << i << " j=" << j;
      }
    }
  }
}
