// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/geodesy.hpp"
#include "ekf/initializer.hpp"
#include "ekf/types.hpp"

namespace ekf {

struct Filter {
  double x[8];
  double P[64];
  double t_s;
  Lifecycle lifecycle;
  std::uint32_t gnss_accepted;
  std::uint32_t gnss_gated;
  std::uint32_t wheel_accepted;
  std::uint32_t wheel_gated;
  std::uint32_t reacquisitions;
  std::uint32_t gnss_consec_gated;
  std::uint32_t wheel_consec_gated;
  double gated_innov[20][2];
  std::uint8_t gated_count;
  InitializerState init;
};

Status ValidateConfig(const Config& cfg);
Status Reset(Filter& f, const Config& cfg, double t0_s);
Status Predict(Filter& f, const Config& cfg, const ImuSample& imu, double dt_s);
UpdateResult UpdateWheel(Filter& f, const Config& cfg, const WheelSample& w);
UpdateResult UpdateGnss(Filter& f, const Config& cfg, const Origin& origin,
                        const GnssSample& gnss);

void FilterMemcpy(Filter& dst, const Filter& src);

}  // namespace ekf
