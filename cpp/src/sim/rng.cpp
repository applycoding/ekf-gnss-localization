// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/rng.hpp"

#include <cmath>
#include <cstdint>

namespace ekf::sim {

namespace {

constexpr std::uint64_t kStreamSalt = 0x9e3779b97f4a7c15ULL;

}  // namespace

std::uint64_t Rng::splitmix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

std::uint64_t Rng::rotl(std::uint64_t x, int k) {
  return (x << k) | (x >> (64 - k));
}

std::uint64_t Rng::next(Xoshiro256& st) {
  const std::uint64_t result = rotl(st.s[1] * 5, 7) * 9;
  const std::uint64_t t = st.s[1] << 17;
  st.s[2] ^= st.s[0];
  st.s[3] ^= st.s[1];
  st.s[1] ^= st.s[2];
  st.s[0] ^= st.s[3];
  st.s[2] ^= t;
  st.s[3] = rotl(st.s[3], 45);
  return result;
}

Rng::Rng(std::uint64_t master_seed) {
  for (int i = 0; i < static_cast<int>(RngStream::Count); ++i) {
    std::uint64_t sm = master_seed + kStreamSalt * static_cast<std::uint64_t>(i + 1);
    Xoshiro256& st = states_[i];
    st.s[0] = splitmix64(sm);
    st.s[1] = splitmix64(sm);
    st.s[2] = splitmix64(sm);
    st.s[3] = splitmix64(sm);
  }
}

double Rng::unit_open(Xoshiro256& st) {
  const std::uint64_t bits = next(st);
  const std::uint64_t mantissa = (bits >> 11) | 1ULL;
  const double u = static_cast<double>(mantissa) * (1.0 / 9007199254740992.0);
  if (u >= 1.0) {
    return 1.0 - 2.220446049250313e-16;
  }
  return u;
}

std::uint64_t Rng::raw_u64(RngStream stream) {
  const int idx = static_cast<int>(stream);
  return next(states_[idx]);
}

double Rng::uniform(RngStream stream, double lo, double hi) {
  const int idx = static_cast<int>(stream);
  const double u = unit_open(states_[idx]);
  return lo + (hi - lo) * u;
}

bool Rng::bernoulli(RngStream stream, double p) {
  const int idx = static_cast<int>(stream);
  const double u = unit_open(states_[idx]);
  return u < p;
}

double Rng::normal(RngStream stream) {
  const int idx = static_cast<int>(stream);
  Xoshiro256& st = states_[idx];

  if (normal_spare_[idx]) {
    normal_spare_[idx] = false;
    return normal_cache_[idx];
  }

  double u1 = 0.0;
  double u2 = 0.0;
  double r = 2.0;
  while (r >= 1.0 || r == 0.0) {
    u1 = 2.0 * unit_open(st) - 1.0;
    u2 = 2.0 * unit_open(st) - 1.0;
    r = u1 * u1 + u2 * u2;
  }
  const double mag = std::sqrt(-2.0 * std::log(r) / r);
  normal_cache_[idx] = u1 * mag;
  normal_spare_[idx] = true;
  return u2 * mag;
}

}  // namespace ekf::sim
