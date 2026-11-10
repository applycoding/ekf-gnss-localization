// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/types.hpp"

namespace ekf {

constexpr int kInitWindow = 5;

struct InitFix {
  double t_s;
  double e;
  double n;
  double s_odom;
  bool valid;
};

struct InitializerState {
  Lifecycle lifecycle;
  int count_a;
  InitFix window_a[kInitWindow];
  int count_b;
  InitFix window_b[kInitWindow];
  double s_odom;
  double s_at_end_a;
  double omega_integral;
  double omega_window_start;
  bool have_window_start;
  double last_wheel_speed;
};

struct InitOutput {
  double x[8];
  double P[64];
  bool ready;
};

void InitStateReset(InitializerState& st);
void InitOnWheel(InitializerState& st, const WheelSample& w);
void InitOnImu(InitializerState& st, const ImuSample& imu, double dt);
Status InitOnGnss(InitializerState& st, const GnssSample& gnss, const Origin& origin,
                   const Config& cfg, InitOutput& out);

}  // namespace ekf
