// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scheduler.hpp"

#include "ekf/mat.hpp"
#include "ekf_sim/baselines.hpp"
#include "ekf_sim/constants.hpp"
#include "ekf_sim/sensors.hpp"
#include "ekf_sim/trajectory.hpp"

#include <cmath>
#include <cstring>

namespace ekf::sim {

namespace {

double envelope_radius_m(const ekf::Filter& f) {
  const double a = f.P[0];
  const double b = 0.5 * (f.P[1] + f.P[8]);
  const double c = f.P[9];
  const double tr = a + c;
  const double det = a * c - b * b;
  const double disc = std::sqrt(std::max(0.0, tr * tr * 0.25 - det));
  const double lambda_max = tr * 0.5 + disc;
  return std::sqrt(std::max(0.0, ekf::kEnvelopeChi2 * lambda_max));
}

double nees_position(const ekf::Filter& f, double err_e, double err_n) {
  const double S[2][2] = {{f.P[0], f.P[1]}, {f.P[8], f.P[9]}};
  const double e[2] = {err_e, err_n};
  double x[2];
  if (!ekf::cholesky_solve_2(S, e, x)) {
    return 0.0;
  }
  return e[0] * x[0] + e[1] * x[1];
}

double nees_full(const ekf::Filter& f, double err_e, double err_n, double err_psi, double err_v,
                 double err_bg) {
  const int idx[5] = {0, 1, 2, 3, 4};
  double S[5][5]{};
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      S[i][j] = f.P[idx[i] * 8 + idx[j]];
    }
  }
  const double e_raw[5] = {err_e, err_n, err_psi, err_v, err_bg};
  double e[5] = {err_e, err_n, ekf::WrapPi(err_psi), err_v, err_bg};
  double x[5];
  if (!ekf::cholesky_solve_5(S, e, x)) {
    return 0.0;
  }
  double nis = 0.0;
  for (int i = 0; i < 5; ++i) {
    nis += e_raw[i] * x[i];
  }
  return nis;
}

void fill_tick_record(TickRecord& rec, const ekf::Filter& f, const TruthState& truth, bool init) {
  rec.est_e_m = init ? f.x[0] : 0.0;
  rec.est_n_m = init ? f.x[1] : 0.0;
  rec.est_psi_rad = init ? f.x[2] : 0.0;
  rec.est_v_m_s = init ? f.x[3] : 0.0;
  rec.est_b_g_rad_s = init ? f.x[4] : 0.0;
  rec.est_k = init ? f.x[5] : 1.0;
  rec.filter_initialized = init;
  rec.p_pe = init ? f.P[0] : 0.0;
  rec.p_pn = init ? f.P[9] : 0.0;
  rec.p_psi = init ? f.P[18] : 0.0;
  rec.envelope_m = init ? envelope_radius_m(f) : 0.0;
  rec.truth_e_m = truth.pe_m;
  rec.truth_n_m = truth.pn_m;
  rec.truth_psi_rad = truth.psi_rad;
}

}  // namespace

Trace RunSimulation(const ScenarioSpec& spec, std::uint64_t seed, bool gate_enabled,
                    MeasurementLog* log) {
  Trace trace{};
  TraceClear(trace);
  TraceReserve(trace);
  trace.seed = seed;
  std::strncpy(trace.scenario_name, spec.name.c_str(), sizeof(trace.scenario_name) - 1);

  if (log != nullptr) {
    log->ticks.resize(static_cast<std::size_t>(kNumTicks));
  }

  ekf::Origin origin{};
  const ekf::Llh origin_llh{kOriginLatRad, kOriginLonRad, kOriginHeightM};
  ekf::MakeOrigin(origin_llh, origin);

  ekf::Config cfg = spec.filter;
  cfg.gate = gate_enabled;

  ekf::Filter filter{};
  ekf::Reset(filter, cfg, 0.0);

  Rng rng(seed);
  SensorSimulator sensors(spec, rng, origin);
  TruthState truth{};

  for (int i = 0; i < kNumTicks; ++i) {
  const double t_s = i * kDtS;
    sensors.evolve_gnss_bias(kDtS);

    const ImuTruth imu_truth = sensors.imu_at_tick(i, truth);
    ekf::ImuSample imu = imu_truth.sample;
    if (filter.lifecycle == ekf::Lifecycle::Initialized) {
      imu.t_s = t_s;
    }
    (void)ekf::Predict(filter, cfg, imu, kDtS);

    TickRecord tick{};
    tick.t_s = t_s;
    tick.meas_gyro_z_rad_s = imu.gyro_z_rad_s;

    bool have_wheel = false;
    ekf::WheelSample wheel{};
    if (i % 2 == 0) {
      wheel = sensors.wheel_at_tick(i, truth);
      wheel.t_s = t_s;
      have_wheel = true;
      tick.meas_wheel_m_s = wheel.speed_m_s;
      tick.have_wheel_meas = true;
      const ekf::UpdateResult wres = ekf::UpdateWheel(filter, cfg, wheel);
      if (log != nullptr) {
        log->ticks[static_cast<std::size_t>(i)].wheel = wheel;
        log->ticks[static_cast<std::size_t>(i)].have_wheel = true;
      }
      if (filter.lifecycle == ekf::Lifecycle::Initialized) {
        WheelRecord wr{};
        wr.t_s = wheel.t_s;
        wr.nis_pre_gate = wres.nis_pre_gate;
        wr.accepted = wres.accepted;
        trace.wheel.push_back(wr);
      }
    }

    GnssRecord grec{};
    grec.t_s = t_s;
    grec.truth_e_m = truth.pe_m;
    grec.truth_n_m = truth.pn_m;

    if (i % 10 == 0) {
      ekf::GnssSample gnss{};
      const GnssTruthLabel label = sensors.gnss_at_tick(i, truth, gnss);
      grec.emitted = label.emitted;
      grec.sim_outlier = label.is_outlier;
      grec.meas_e_m = label.meas_e_m;
      grec.meas_n_m = label.meas_n_m;

      if (label.emitted) {
        const ekf::UpdateResult gres = ekf::UpdateGnss(filter, cfg, origin, gnss);
        grec.nis_pre_gate = gres.nis_pre_gate;
        grec.accepted = gres.accepted;
        grec.update_status = static_cast<std::uint8_t>(gres.status);

        if (trace.init_time_s < 0.0 && filter.lifecycle == ekf::Lifecycle::Initialized) {
          trace.init_time_s = filter.t_s;
        }

        if (filter.lifecycle == ekf::Lifecycle::Initialized) {
          const double err_e = filter.x[0] - truth.pe_m;
          const double err_n = filter.x[1] - truth.pn_m;
          grec.nees_pos = nees_position(filter, err_e, err_n);
          grec.nees_full = nees_full(filter, err_e, err_n, filter.x[2] - truth.psi_rad,
                                    filter.x[3] - truth.v_m_s, filter.x[4] - spec.gyro_bias_rad_s);
        }
        trace.gnss.push_back(grec);
      }

      if (log != nullptr) {
        log->ticks[static_cast<std::size_t>(i)].have_gnss = label.emitted;
        log->ticks[static_cast<std::size_t>(i)].gnss = gnss;
        log->ticks[static_cast<std::size_t>(i)].gnss_outlier = label.is_outlier;
        log->ticks[static_cast<std::size_t>(i)].gnss_meas_e = label.meas_e_m;
        log->ticks[static_cast<std::size_t>(i)].gnss_meas_n = label.meas_n_m;
      }
    }

    integrate_truth_midpoint(truth, t_s, kDtS);

    fill_tick_record(tick, filter, truth, filter.lifecycle == ekf::Lifecycle::Initialized);
    trace.ticks.push_back(tick);

    if (log != nullptr) {
      auto& m = log->ticks[static_cast<std::size_t>(i)];
      m.imu = imu;
      m.truth_e = truth.pe_m;
      m.truth_n = truth.pn_m;
      m.truth_psi = truth.psi_rad;
      if (!m.have_wheel && have_wheel) {
        m.wheel = wheel;
        m.have_wheel = have_wheel;
      }
    }
  }

  trace.gnss_accepted = filter.gnss_accepted;
  trace.gnss_gated = filter.gnss_gated;
  trace.wheel_accepted = filter.wheel_accepted;
  trace.wheel_gated = filter.wheel_gated;
  trace.reacquisitions = filter.reacquisitions;
  if (filter.lifecycle == ekf::Lifecycle::Initialized) {
    trace.final_b_g_rad_s = filter.x[4];
    trace.final_k = filter.x[5];
  }

  return trace;
}

}  // namespace ekf::sim
