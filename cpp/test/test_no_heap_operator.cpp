// SPDX-License-Identifier: Apache-2.0
#include "ekf/filter.hpp"
#include "ekf/geodesy.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>

void* operator new(std::size_t) { std::abort(); }
void* operator new[](std::size_t) { std::abort(); }
void operator delete(void*) noexcept {}
void operator delete[](void*) noexcept {}
void* operator new(std::size_t, std::align_val_t) { std::abort(); }
void* operator new[](std::size_t, std::align_val_t) { std::abort(); }
void operator delete(void*, std::align_val_t) noexcept {}
void operator delete[](void*, std::align_val_t) noexcept {}
void* operator new(std::size_t, const std::nothrow_t&) noexcept { return nullptr; }
void* operator new[](std::size_t, const std::nothrow_t&) noexcept { return nullptr; }

namespace {

int fail(const char* msg) {
  std::fprintf(stderr, "%s\n", msg);
  return 1;
}

ekf::Config base_cfg() {
  ekf::Config c{};
  c.q_gyro = 4e-6;
  c.q_accel = 2.5e-3;
  c.q_bias = 1e-8;
  c.q_scale = 1e-10;
  c.r_wheel = 0.0025;
  c.gnss_sigma_m = 1.5;
  c.gnss_bias_sigma_m = 1.0;
  c.gnss_bias_tau_s = 60.0;
  c.model_gnss_bias = true;
  c.estimate_wheel_scale = true;
  c.estimate_gyro_bias = true;
  c.gate = true;
  return c;
}

}  // namespace

int main() {
  ekf::Filter f{};
  const ekf::Config cfg = base_cfg();
  if (ekf::Reset(f, cfg, 0.0) != ekf::Status::Ok) {
    return fail("reset");
  }
  f.lifecycle = ekf::Lifecycle::Initialized;
  f.t_s = 1.0;
  for (int i = 0; i < 8; ++i) {
    f.x[i] = 0.0;
    f.P[i * 8 + i] = 1.0;
  }
  f.x[3] = 5.0;
  const ekf::ImuSample imu{1.01, 0.01, 0.0};
  if (ekf::Predict(f, cfg, imu, 0.01) != ekf::Status::Ok) {
    return fail("predict");
  }
  ekf::WheelSample wheel{1.01, 5.0};
  if (ekf::UpdateWheel(f, cfg, wheel).status != ekf::Status::Ok) {
    return fail("wheel");
  }
  ekf::Origin origin{};
  if (ekf::MakeOrigin({0.0, 0.0, 0.0}, origin) != ekf::Status::Ok) {
    return fail("origin");
  }
  ekf::GnssSample gnss{1.01, 0.0, 0.0, 0.0};
  if (ekf::UpdateGnss(f, cfg, origin, gnss).status != ekf::Status::Ok) {
    return fail("gnss");
  }
  const ekf::Filter backup = f;
  ekf::ImuSample bad_imu{1.02, std::numeric_limits<double>::quiet_NaN(), 0.0};
  if (ekf::Predict(f, cfg, bad_imu, 0.01) != ekf::Status::InvalidInput) {
    return fail("invalid");
  }
  if (std::memcmp(&backup, &f, sizeof(f)) != 0) {
    return fail("unchanged");
  }
  return 0;
}
