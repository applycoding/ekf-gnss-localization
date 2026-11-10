// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/filter.hpp"
#include "ekf_sim/scenario.hpp"
#include "ekf_sim/trace.hpp"

namespace ekf::sim {

struct MeasurementLog {
  struct TickMeas {
    ekf::ImuSample imu{};
    bool have_wheel = false;
    ekf::WheelSample wheel{};
    bool have_gnss = false;
    ekf::GnssSample gnss{};
    bool gnss_outlier = false;
    double gnss_meas_e = 0.0;
    double gnss_meas_n = 0.0;
    double truth_e = 0.0;
    double truth_n = 0.0;
    double truth_psi = 0.0;
  };
  std::vector<TickMeas> ticks;
};

/** Run filter + truth sim; fill trace and optional measurement log for replay. */
Trace RunSimulation(const ScenarioSpec& spec, std::uint64_t seed, bool gate_enabled,
                    MeasurementLog* log);

}  // namespace ekf::sim
