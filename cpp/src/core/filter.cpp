// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"

#include "ekf/mat.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace ekf {

void FilterMemcpy(Filter& dst, const Filter& src) {
  std::memcpy(&dst, &src, sizeof(Filter));
}

namespace {

bool is_finite_filter(const Filter& f) {
  for (int i = 0; i < 8; ++i) {
    if (!std::isfinite(f.x[i])) {
      return false;
    }
  }
  for (int i = 0; i < 64; ++i) {
    if (!std::isfinite(f.P[i])) {
      return false;
    }
  }
  return true;
}

bool is_pd_diagonal(const Filter& f, const Config& cfg) {
  constexpr int kDiagIdx[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  for (int i = 0; i < 8; ++i) {
    if (i == 5 && !cfg.estimate_wheel_scale) {
      continue;
    }
    if ((i == 6 || i == 7) && !cfg.model_gnss_bias) {
      continue;
    }
    if (!(f.P[kDiagIdx[i] * 9] > 0.0)) {
      return false;
    }
  }
  return f.P[0] > 0 && f.P[9] > 0 && f.P[18] > 0 && f.P[27] > 0 && f.P[36] > 0 &&
         (cfg.estimate_wheel_scale ? f.P[45] > 0 : true) &&
         (cfg.model_gnss_bias ? (f.P[54] > 0 && f.P[63] > 0) : true);
}

void symmetrize_p(double P[64]) {
  for (int i = 0; i < 8; ++i) {
    for (int j = i + 1; j < 8; ++j) {
      const double avg = 0.5 * (P[i * 8 + j] + P[j * 8 + i]);
      P[i * 8 + j] = avg;
      P[j * 8 + i] = avg;
    }
  }
}

bool time_match(double t_f, double t_meas) {
  return std::abs(t_f - t_meas) <= 1e-9;
}

}  // namespace

Status ValidateConfig(const Config& cfg) {
  if (!(cfg.q_gyro > 0) || !(cfg.q_accel > 0) || !(cfg.q_bias > 0) || !(cfg.q_scale > 0)) {
    return Status::InvalidInput;
  }
  if (!(cfg.r_wheel > 0) || !(cfg.gnss_sigma_m > 0)) {
    return Status::InvalidInput;
  }
  if (cfg.model_gnss_bias && (!(cfg.gnss_bias_sigma_m > 0) || !(cfg.gnss_bias_tau_s > 0))) {
    return Status::InvalidInput;
  }
  return Status::Ok;
}

Status Reset(Filter& f, const Config& cfg, double t0_s) {
  if (ValidateConfig(cfg) != Status::Ok || !std::isfinite(t0_s)) {
    return Status::InvalidInput;
  }
  std::memset(&f, 0, sizeof(f));
  f.t_s = t0_s;
  f.lifecycle = Lifecycle::NoFix;
  InitStateReset(f.init);
  f.init.lifecycle = Lifecycle::NoFix;
  return Status::Ok;
}

Status Predict(Filter& f, const Config& cfg, const ImuSample& imu, double dt_s) {
  const Filter backup = f;
  if (ValidateConfig(cfg) != Status::Ok) {
    return Status::InvalidInput;
  }
  if (!(dt_s > 0.0) || dt_s > 0.02 || !std::isfinite(dt_s)) {
    return Status::InvalidDt;
  }
  if (!std::isfinite(imu.gyro_z_rad_s) || !std::isfinite(imu.accel_x_m_s2) ||
      !std::isfinite(imu.t_s)) {
    return Status::InvalidInput;
  }
  if (!time_match(f.t_s + dt_s, imu.t_s) && f.lifecycle == Lifecycle::Initialized) {
    return Status::TimeMismatch;
  }

  InitOnImu(f.init, imu, dt_s);

  if (f.lifecycle != Lifecycle::Initialized) {
    return Status::NotInitialized;
  }

  const double psi = f.x[2];
  const double v = f.x[3];
  const double c = std::cos(psi);
  const double s = std::sin(psi);

  f.x[0] += v * c * dt_s;
  f.x[1] += v * s * dt_s;
  f.x[2] = WrapPi(f.x[2] + (imu.gyro_z_rad_s - f.x[4]) * dt_s);
  f.x[3] += imu.accel_x_m_s2 * dt_s;

  const double phi =
      cfg.model_gnss_bias ? std::exp(-dt_s / cfg.gnss_bias_tau_s) : 0.0;
  if (cfg.model_gnss_bias) {
    f.x[6] *= phi;
    f.x[7] *= phi;
  } else {
    f.x[6] = 0.0;
    f.x[7] = 0.0;
  }

  double F[64];
  std::memset(F, 0, sizeof(F));
  for (int i = 0; i < 8; ++i) {
    F[i * 8 + i] = 1.0;
  }
  F[0 * 8 + 2] = -v * s * dt_s;
  F[0 * 8 + 3] = c * dt_s;
  F[1 * 8 + 2] = v * c * dt_s;
  F[1 * 8 + 3] = s * dt_s;
  F[2 * 8 + 4] = -dt_s;
  if (cfg.model_gnss_bias) {
    F[6 * 8 + 6] = phi;
    F[7 * 8 + 7] = phi;
  }

  double FP[64];
  std::memset(FP, 0, sizeof(FP));
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        sum += F[i * 8 + k] * backup.P[k * 8 + j];
      }
      FP[i * 8 + j] = sum;
    }
  }
  double Pnew[64];
  std::memset(Pnew, 0, sizeof(Pnew));
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        sum += FP[i * 8 + k] * F[j * 8 + k];
      }
      Pnew[i * 8 + j] = sum;
    }
  }

  Pnew[0 * 8 + 0] += 0.0;
  Pnew[1 * 8 + 1] += 0.0;
  Pnew[2 * 8 + 2] += cfg.q_gyro * dt_s;
  Pnew[3 * 8 + 3] += cfg.q_accel * dt_s;
  Pnew[4 * 8 + 4] += cfg.q_bias * dt_s;
  if (cfg.estimate_wheel_scale) {
    Pnew[5 * 8 + 5] += cfg.q_scale * dt_s;
  }
  if (cfg.model_gnss_bias) {
    const double sb2 = cfg.gnss_bias_sigma_m * cfg.gnss_bias_sigma_m;
    const double q_b = sb2 * (1.0 - phi * phi);
    Pnew[6 * 8 + 6] += q_b;
    Pnew[7 * 8 + 7] += q_b;
  }

  if (!cfg.model_gnss_bias) {
    for (int i = 0; i < 8; ++i) {
      f.P[i * 8 + 6] = 0.0;
      f.P[i * 8 + 7] = 0.0;
      f.P[6 * 8 + i] = 0.0;
      f.P[7 * 8 + i] = 0.0;
    }
    f.x[6] = 0.0;
    f.x[7] = 0.0;
  }

  std::memcpy(f.P, Pnew, sizeof(Pnew));
  symmetrize_p(f.P);
  f.t_s += dt_s;

  if (!is_finite_filter(f) || !is_pd_diagonal(f, cfg)) {
    FilterMemcpy(f, backup);
    return Status::Numerical;
  }
  return Status::Ok;
}

namespace {

UpdateResult joseph_scalar_update(Filter& f, const Config& cfg, int state_idx, double z,
                                  double R, double gate_thresh, bool use_gate,
                                  std::uint32_t& accepted, std::uint32_t& gated,
                                  std::uint32_t& consec_gated, bool wheel_lockout,
                                  double& y_store) {
  const Filter backup = f;
  UpdateResult res{};
  res.nis_pre_gate = std::numeric_limits<double>::quiet_NaN();
  res.accepted = false;

  const double h_val = (state_idx == 3) ? f.x[5] * f.x[3] : f.x[state_idx];
  const double y = z - h_val;
  y_store = y;
  double S = 0.0;
  if (state_idx == 3 && cfg.estimate_wheel_scale) {
    S = f.P[27] * f.x[5] * f.x[5] + 2.0 * f.P[27 * 0 + 5] * f.x[3] * f.x[5] + f.P[45] * f.x[3] * f.x[3] + R;
  } else if (state_idx == 3) {
    S = f.x[5] * f.x[5] * f.P[27] + R;
  } else {
    S = f.P[state_idx * 8 + state_idx] + R;
  }

  if (!(S > 0.0) || !std::isfinite(S)) {
    FilterMemcpy(f, backup);
    res.status = Status::Numerical;
    return res;
  }
  res.nis_pre_gate = (y * y) / S;

  bool apply = true;
  if (use_gate && cfg.gate && res.nis_pre_gate > gate_thresh) {
    apply = false;
    if (wheel_lockout) {
      ++consec_gated;
      if (consec_gated >= 100) {
        f.P[27] += y * y;
        apply = true;
        consec_gated = 0;
      }
    } else {
      ++consec_gated;
    }
    if (!apply) {
      ++gated;
      res.status = Status::Gated;
      res.accepted = false;
      return res;
    }
  } else {
    consec_gated = 0;
  }

  double K[8];
  std::memset(K, 0, sizeof(K));
  for (int i = 0; i < 8; ++i) {
    if (state_idx == 3 && cfg.estimate_wheel_scale) {
      K[i] = (f.P[i * 8 + 3] * f.x[5] + f.P[i * 8 + 5] * f.x[3]) / S;
    } else if (state_idx == 3) {
      K[i] = f.P[i * 8 + 3] * f.x[5] / S;
    } else {
      K[i] = f.P[i * 8 + state_idx] / S;
    }
  }

  for (int i = 0; i < 8; ++i) {
    f.x[i] += K[i] * y;
  }
  if (state_idx == 2 || true) {
    f.x[2] = WrapPi(f.x[2]);
  }

  double IKH[64];
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double hij = (j == state_idx) ? 1.0 : 0.0;
      if (state_idx == 3 && cfg.estimate_wheel_scale) {
        if (j == 3) {
          hij = f.x[5];
        } else if (j == 5) {
          hij = f.x[3];
        }
      } else if (state_idx == 3 && j == 3) {
        hij = f.x[5];
      }
      IKH[i * 8 + j] = (i == j ? 1.0 : 0.0) - K[i] * hij;
    }
  }

  double Pmid[64];
  std::memset(Pmid, 0, sizeof(Pmid));
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        sum += IKH[i * 8 + k] * backup.P[k * 8 + j];
      }
      Pmid[i * 8 + j] = sum;
    }
  }
  double Pnew[64];
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        sum += Pmid[i * 8 + k] * IKH[j * 8 + k];
      }
      Pnew[i * 8 + j] = sum;
    }
  }
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      if (state_idx == 3 && j == 3) {
        (void)f.x[5];
      }
      Pnew[i * 8 + j] += K[i] * R * K[j];
    }
  }
  std::memcpy(f.P, Pnew, sizeof(Pnew));
  symmetrize_p(f.P);

  if (!is_finite_filter(f) || !is_pd_diagonal(f, cfg)) {
    FilterMemcpy(f, backup);
    res.status = Status::Numerical;
    return res;
  }

  ++accepted;
  res.status = Status::Ok;
  res.accepted = true;
  return res;
}

}  // namespace

UpdateResult UpdateWheel(Filter& f, const Config& cfg, const WheelSample& w) {
  const Filter backup = f;
  UpdateResult res{};
  res.nis_pre_gate = std::numeric_limits<double>::quiet_NaN();
  if (f.lifecycle != Lifecycle::Initialized) {
    InitOnWheel(f.init, w);
    res.status = Status::NotInitialized;
    return res;
  }
  if (!std::isfinite(w.speed_m_s) || !std::isfinite(w.t_s)) {
    res.status = Status::InvalidInput;
    return res;
  }
  if (!time_match(f.t_s, w.t_s)) {
    res.status = Status::TimeMismatch;
    return res;
  }

  double y = 0.0;
  res = joseph_scalar_update(f, cfg, 3, w.speed_m_s, cfg.r_wheel, kWheelGateNis, true,
                             f.wheel_accepted, f.wheel_gated, f.wheel_consec_gated, true, y);
  if (res.status == Status::InvalidInput || res.status == Status::TimeMismatch) {
    FilterMemcpy(f, backup);
  }
  return res;
}

UpdateResult UpdateGnss(Filter& f, const Config& cfg, const Origin& origin,
                        const GnssSample& gnss) {
  const Filter backup = f;
  UpdateResult res{};
  res.nis_pre_gate = std::numeric_limits<double>::quiet_NaN();

  InitOutput init_out{};
  const Status init_st =
      InitOnGnss(f.init, gnss, origin, cfg, init_out);
  if (init_st != Status::Ok) {
    res.status = init_st;
    return res;
  }
  if (init_out.ready && f.lifecycle != Lifecycle::Initialized) {
    std::memcpy(f.x, init_out.x, sizeof(f.x));
    std::memcpy(f.P, init_out.P, sizeof(f.P));
    f.lifecycle = Lifecycle::Initialized;
    f.t_s = gnss.t_s;
    res.status = Status::Ok;
    res.accepted = true;
    return res;
  }

  if (f.lifecycle != Lifecycle::Initialized) {
    res.status = Status::NotInitialized;
    return res;
  }
  if (!std::isfinite(gnss.lat_rad) || !std::isfinite(gnss.lon_rad) || !std::isfinite(gnss.h_m) ||
      !std::isfinite(gnss.t_s)) {
    FilterMemcpy(f, backup);
    res.status = Status::InvalidInput;
    return res;
  }
  if (!time_match(f.t_s, gnss.t_s)) {
    FilterMemcpy(f, backup);
    res.status = Status::TimeMismatch;
    return res;
  }

  double ecef[3];
  const Llh llh{gnss.lat_rad, gnss.lon_rad, gnss.h_m};
  GeodeticToEcef(llh, ecef);
  Enu enu{};
  EcefToEnu(origin, ecef, enu);
  const double z[2] = {enu.e, enu.n};

  const double R = cfg.gnss_sigma_m * cfg.gnss_sigma_m;
  double S[2][2];
  if (cfg.model_gnss_bias) {
    S[0][0] = f.P[0] + f.P[6] + f.P[48] + f.P[54] + R;
    S[0][1] = f.P[1] + f.P[7] + f.P[49] + f.P[55];
    S[1][0] = f.P[8] + f.P[14] + f.P[56] + f.P[62];
    S[1][1] = f.P[9] + f.P[15] + f.P[57] + f.P[63] + R;
  } else {
    S[0][0] = f.P[0] + R;
    S[0][1] = f.P[1];
    S[1][0] = f.P[8];
    S[1][1] = f.P[9] + R;
  }

  const double innov[2] = {z[0] - (f.x[0] + (cfg.model_gnss_bias ? f.x[6] : 0.0)),
                           z[1] - (f.x[1] + (cfg.model_gnss_bias ? f.x[7] : 0.0))};

  double Sinv_innov[2];
  if (!cholesky_solve_2(S, innov, Sinv_innov)) {
    FilterMemcpy(f, backup);
    res.status = Status::Numerical;
    return res;
  }
  double nis = innov[0] * Sinv_innov[0] + innov[1] * Sinv_innov[1];
  res.nis_pre_gate = nis;

  bool apply = true;
  if (cfg.gate && nis > kGnssGateNis) {
    apply = false;
    ++f.gnss_gated;
    ++f.gnss_consec_gated;
    if (f.gated_count < 20) {
      f.gated_innov[f.gated_count][0] = innov[0];
      f.gated_innov[f.gated_count][1] = innov[1];
      ++f.gated_count;
    }
    if (f.gnss_consec_gated >= 20) {
      double med[2] = {0.0, 0.0};
      for (int i = 0; i < 20; ++i) {
        med[0] += f.gated_innov[i][0];
        med[1] += f.gated_innov[i][1];
      }
      med[0] /= 20.0;
      med[1] /= 20.0;
      bool consistent = true;
      const double thr = 3.0 * std::sqrt(2.0) * cfg.gnss_sigma_m;
      for (int i = 0; i < 20; ++i) {
        if (std::abs(f.gated_innov[i][0] - med[0]) > thr ||
            std::abs(f.gated_innov[i][1] - med[1]) > thr) {
          consistent = false;
          break;
        }
      }
      if (consistent) {
        f.P[0] += nis * R;
        f.P[9] += nis * R;
        apply = true;
        ++f.reacquisitions;
        f.gnss_consec_gated = 0;
        f.gated_count = 0;
        res.status = Status::Reacquired;
      } else {
        f.gnss_consec_gated = 0;
        f.gated_count = 0;
      }
    }
    if (!apply) {
      res.status = Status::Gated;
      res.accepted = false;
      return res;
    }
  } else {
    f.gnss_consec_gated = 0;
    f.gated_count = 0;
  }

  double K[8][2];
  std::memset(K, 0, sizeof(K));
  for (int i = 0; i < 8; ++i) {
    double ph[2] = {0.0, 0.0};
    for (int k = 0; k < 8; ++k) {
      double h0 = 0.0;
      double h1 = 0.0;
      if (k == 0) {
        h0 = 1.0;
      }
      if (k == 1) {
        h1 = 1.0;
      }
      if (cfg.model_gnss_bias) {
        if (k == 6) {
          h0 = 1.0;
        }
        if (k == 7) {
          h1 = 1.0;
        }
      }
      ph[0] += f.P[i * 8 + k] * h0;
      ph[1] += f.P[i * 8 + k] * h1;
    }
    double sol[2];
    if (!cholesky_solve_2(S, ph, sol)) {
      FilterMemcpy(f, backup);
      res.status = Status::Numerical;
      return res;
    }
    K[i][0] = sol[0];
    K[i][1] = sol[1];
  }

  for (int i = 0; i < 8; ++i) {
    f.x[i] += K[i][0] * innov[0] + K[i][1] * innov[1];
  }
  f.x[2] = WrapPi(f.x[2]);

  double H[2][8] = {};
  H[0][0] = 1.0;
  H[1][1] = 1.0;
  if (cfg.model_gnss_bias) {
    H[0][6] = 1.0;
    H[1][7] = 1.0;
  }

  double IKH[8][8];
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      IKH[i][j] = (i == j ? 1.0 : 0.0);
      for (int c = 0; c < 2; ++c) {
        IKH[i][j] -= K[i][c] * H[c][j];
      }
    }
  }
  double Pnew[64];
  std::memset(Pnew, 0, sizeof(Pnew));
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 8; ++k) {
        double mid = 0.0;
        for (int l = 0; l < 8; ++l) {
          mid += IKH[i][l] * backup.P[l * 8 + k];
        }
        sum += mid * IKH[j][k];
      }
      Pnew[i * 8 + j] = sum;
    }
  }
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      Pnew[i * 8 + j] += K[i][0] * R * K[j][0] + K[i][1] * R * K[j][1];
    }
  }
  std::memcpy(f.P, Pnew, sizeof(Pnew));
  symmetrize_p(f.P);

  if (!is_finite_filter(f) || !is_pd_diagonal(f, cfg)) {
    FilterMemcpy(f, backup);
    res.status = Status::Numerical;
    return res;
  }

  ++f.gnss_accepted;
  res.status = res.status == Status::Reacquired ? Status::Reacquired : Status::Ok;
  res.accepted = true;
  return res;
}

}  // namespace ekf
