// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

namespace ekf::sim {

enum class RngStream : std::uint8_t {
  Gyro = 0,
  Accel,
  Wheel,
  GnssWhiteE,
  GnssWhiteN,
  GnssGm,
  OutlierDraw,
  OutlierAngle,
  OutlierLength,
  Count
};

class Rng {
 public:
  explicit Rng(std::uint64_t master_seed);

  double normal(RngStream stream);
  /** Uniform on [lo, hi) using portable 53-bit mantissa; never libc++ distributions. */
  double uniform(RngStream stream, double lo, double hi);
  /** Bernoulli with probability p in [0, 1]. */
  bool bernoulli(RngStream stream, double p);

  std::uint64_t raw_u64(RngStream stream);

 private:
  struct Xoshiro256 {
    std::uint64_t s[4]{};
  };
  Xoshiro256 states_[static_cast<int>(RngStream::Count)];

  static std::uint64_t splitmix64(std::uint64_t& state);
  static std::uint64_t rotl(std::uint64_t x, int k);
  static std::uint64_t next(Xoshiro256& st);
  static double unit_open(Xoshiro256& st);
  bool normal_spare_[static_cast<int>(RngStream::Count)]{};
  double normal_cache_[static_cast<int>(RngStream::Count)]{};
};

}  // namespace ekf::sim
