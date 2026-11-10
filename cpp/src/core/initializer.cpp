// SPDX-License-Identifier: Apache-2.0
#include "ekf/initializer.hpp"

#include "ekf/geodesy.hpp"

#include <cmath>
#include <cstring>

namespace ekf {

namespace {

double median_component(const InitFix* fixes, int n, bool east) {
  double vals[kInitWindow];
  for (int i = 0; i < n; ++i) {
    vals[i] = east ? fixes[i].e : fixes[i].n;
  }
  for (int i = 1; i < n; ++i) {
    const double key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      --j;
    }
    vals[j + 1] = key;
  }
  if (n % 2 == 1) {
    return vals[n / 2];
  }
  return 0.5 * (vals[n / 2 - 1] + vals[n / 2]);
}

void shift_fixes(InitFix* w, int n) {
  for (int i = 0; i < n - 1; ++i) {
    w[i] = w[i + 1];
  }
}

}  // namespace

void InitStateReset(InitializerState& st) {
  std::memset(&st, 0, sizeof(st));
  st.lifecycle = Lifecycle::NoFix;
}

void InitOnWheel(InitializerState& st, const WheelSample& w) {
  st.last_wheel_speed = w.speed_m_s;
  st.s_odom += w.speed_m_s * 0.02;
}

void InitOnImu(InitializerState& st, const ImuSample& imu, double dt) {
  if (st.lifecycle == Lifecycle::Collecting && st.have_window_start) {
    st.omega_integral += imu.gyro_z_rad_s * dt;
  }
}

Status InitOnGnss(InitializerState& st, const GnssSample& gnss, const Origin& origin,
                  const Config& cfg, InitOutput& out) {
  out.ready = false;
  double ecef[3];
  const Llh llh{gnss.lat_rad, gnss.lon_rad, gnss.h_m};
  GeodeticToEcef(llh, ecef);
  Enu enu{};
  EcefToEnu(origin, ecef, enu);

  InitFix fix{gnss.t_s, enu.e, enu.n, st.s_odom, true};

  if (st.lifecycle == Lifecycle::NoFix) {
    st.lifecycle = Lifecycle::Collecting;
    st.count_a = 0;
    st.count_b = 0;
    st.s_odom = 0.0;
    st.omega_integral = 0.0;
    st.have_window_start = false;
  }

  if (st.lifecycle != Lifecycle::Collecting) {
    return Status::Ok;
  }

  if (st.count_a < kInitWindow) {
    st.window_a[st.count_a++] = fix;
    if (st.count_a == kInitWindow) {
      st.s_at_end_a = st.s_odom;
      st.omega_integral = 0.0;
      st.have_window_start = true;
    }
    return Status::Ok;
  }

  if (st.s_odom - st.s_at_end_a >= 20.0) {
    if (st.count_b < kInitWindow) {
      st.window_b[st.count_b++] = fix;
    } else {
      shift_fixes(st.window_b, kInitWindow);
      st.window_b[kInitWindow - 1] = fix;
    }
  }

  if (st.count_b < kInitWindow) {
    return Status::Ok;
  }

  if (std::abs(st.omega_integral) >= 0.1) {
    st.count_a = 0;
    st.count_b = 0;
    st.window_a[0] = fix;
    st.count_a = 1;
    st.s_at_end_a = st.s_odom;
    st.omega_integral = 0.0;
    st.have_window_start = true;
    return Status::Ok;
  }

  if (st.last_wheel_speed <= 0.0) {
    return Status::Ok;
  }

  const double de_a = median_component(st.window_a, kInitWindow, true);
  const double dn_a = median_component(st.window_a, kInitWindow, false);
  const double de_b = median_component(st.window_b, kInitWindow, true);
  const double dn_b = median_component(st.window_b, kInitWindow, false);
  const double delta_e = de_b - de_a;
  const double delta_n = dn_b - dn_a;
  const double delta = std::hypot(delta_e, delta_n);
  const double sigma_meas = cfg.gnss_sigma_m;
  const double delta_denom = delta < 1e-3 ? 1e-3 : delta;
  double sigma_psi = 3.0 * sigma_meas / delta_denom;
  if (sigma_psi < 0.05) {
    sigma_psi = 0.05;
  }
  if (sigma_psi > 0.5) {
    sigma_psi = 0.5;
  }
  const double psi0 = std::atan2(delta_n, delta_e);
  const double c0 = std::cos(psi0);
  const double s0 = std::sin(psi0);
  const double s_end = st.window_b[kInitWindow - 1].s_odom;

  double pe_vals[kInitWindow];
  double pn_vals[kInitWindow];
  for (int j = 0; j < kInitWindow; ++j) {
    const double ds = s_end - st.window_b[j].s_odom;
    pe_vals[j] = st.window_b[j].e + ds * c0;
    pn_vals[j] = st.window_b[j].n + ds * s0;
  }
  auto median_sorted = [](double* vals, int n) {
    for (int i = 1; i < n; ++i) {
      const double key = vals[i];
      int j = i - 1;
      while (j >= 0 && vals[j] > key) {
        vals[j + 1] = vals[j];
        --j;
      }
      vals[j + 1] = key;
    }
    if (n % 2 == 1) {
      return vals[n / 2];
    }
    return 0.5 * (vals[n / 2 - 1] + vals[n / 2]);
  };
  const double pe_m = median_sorted(pe_vals, kInitWindow);
  const double pn_m = median_sorted(pn_vals, kInitWindow);

  std::memset(out.P, 0, sizeof(out.P));
  const double sigma_p2 =
      cfg.model_gnss_bias
          ? (cfg.gnss_sigma_m * cfg.gnss_sigma_m + cfg.gnss_bias_sigma_m * cfg.gnss_bias_sigma_m)
          : (cfg.gnss_sigma_m * cfg.gnss_sigma_m);
  out.x[0] = pe_m;
  out.x[1] = pn_m;
  out.x[2] = psi0;
  out.x[3] = st.last_wheel_speed;
  out.x[4] = 0.0;
  out.x[5] = 1.0;
  out.x[6] = 0.0;
  out.x[7] = 0.0;

  out.P[0] = sigma_p2;
  out.P[9] = sigma_p2;
  out.P[18] = sigma_psi * sigma_psi;
  out.P[27] = 0.25;
  out.P[36] = 0.02 * 0.02;
  out.P[45] = 0.03 * 0.03;
  if (cfg.model_gnss_bias) {
    const double sb2 = cfg.gnss_bias_sigma_m * cfg.gnss_bias_sigma_m;
    out.P[54] = sb2;
    out.P[63] = sb2;
    out.P[6] = -sb2;
    out.P[15] = -sb2;
    out.P[48] = -sb2;
    out.P[51] = -sb2;
  }

  out.ready = true;
  st.lifecycle = Lifecycle::Initialized;
  return Status::Ok;
}

}  // namespace ekf
