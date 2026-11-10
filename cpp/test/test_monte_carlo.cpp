// SPDX-License-Identifier: Apache-2.0
#include "ekf_sim/scenario.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace {

struct NeesBlock {
  double time_mean = 0.0;
  double fraction_in_bounds = 0.0;
};

NeesBlock aggregate_nees(const char* scenario, int seed_begin, int seed_end, bool use_full) {
  constexpr int kGnssRateHz = 10;
  constexpr double kDurationS = 125.0;
  constexpr std::size_t kMaxEpochs = static_cast<std::size_t>(kDurationS * kGnssRateHz) + 1;
  std::vector<std::vector<double>> per_epoch(kMaxEpochs);
  for (int seed = seed_begin; seed <= seed_end; ++seed) {
    const auto r = ekf::sim::RunScenario(scenario, static_cast<std::uint64_t>(seed));
    const double t0 = r.metrics.init_time_s + 5.0;
    for (const auto& g : r.trace.gnss) {
      const double nees_val = use_full ? g.nees_full : g.nees_pos;
      if (g.t_s + 1e-9 < t0 || !g.accepted || nees_val <= 0.0) {
        continue;
      }
      const std::size_t idx = static_cast<std::size_t>(g.t_s * kGnssRateHz + 0.5);
      if (idx < kMaxEpochs) {
        per_epoch[idx].push_back(nees_val);
      }
    }
  }
  double sum_epoch_means = 0.0;
  std::size_t epochs_used = 0;
  std::size_t epochs_in_bounds = 0;
  for (const auto& bucket : per_epoch) {
    if (bucket.empty()) {
      continue;
    }
    double mean = 0.0;
    for (double v : bucket) {
      mean += v;
    }
    mean /= static_cast<double>(bucket.size());
    sum_epoch_means += mean;
    ++epochs_used;
    const double lo = use_full ? ekf::kNeessFullLower : ekf::kNeessPosLower;
    const double hi = use_full ? ekf::kNeessFullUpper : ekf::kNeessPosUpper;
    if (mean >= lo && mean <= hi) {
      ++epochs_in_bounds;
    }
  }
  NeesBlock out{};
  if (epochs_used > 0) {
    out.time_mean = sum_epoch_means / static_cast<double>(epochs_used);
    out.fraction_in_bounds = static_cast<double>(epochs_in_bounds) / static_cast<double>(epochs_used);
  }
  return out;
}

}  // namespace

TEST(MonteCarlo, MatchedPositionNeessN50) {
  const NeesBlock b = aggregate_nees("matched", 1, 50, false);
  EXPECT_GE(b.time_mean, ekf::kNeessPosLower);
  EXPECT_LE(b.time_mean, ekf::kNeessPosUpper);
  EXPECT_GE(b.fraction_in_bounds, 0.80);
}

TEST(MonteCarlo, MatchedFullNeessN50) {
  const NeesBlock b = aggregate_nees("matched", 1, 50, true);
  EXPECT_GE(b.time_mean, ekf::kNeessFullLower);
  EXPECT_LE(b.time_mean, ekf::kNeessFullUpper);
  EXPECT_GE(b.fraction_in_bounds, 0.70);
}

TEST(MonteCarlo, UrbanPositionNeessN50) {
  const NeesBlock b = aggregate_nees("urban", 1, 50, false);
  EXPECT_GE(b.time_mean, ekf::kNeessPosLower);
  EXPECT_LE(b.time_mean, ekf::kNeessPosUpper);
  EXPECT_GE(b.fraction_in_bounds, 0.80);
}

TEST(MonteCarlo, UrbanWhiteModelFailsNeessN50) {
  const NeesBlock b = aggregate_nees("urban_white_model", 1, 50, false);
  EXPECT_GT(b.time_mean, 5.18);
}

TEST(MonteCarlo, MatchedGnssRejectionRateN50) {
  double sum = 0.0;
  for (int seed = 1; seed <= 50; ++seed) {
    const auto r = ekf::sim::RunScenario("matched", static_cast<std::uint64_t>(seed));
    sum += r.metrics.gnss_rejection_rate;
  }
  const double mean = sum / 50.0;
  EXPECT_GE(mean, 0.005);
  EXPECT_LE(mean, 0.02);
}

TEST(MonteCarlo, UrbanOutlierRecallAndFalseRejectN50) {
  double recall_sum = 0.0;
  double false_reject_sum = 0.0;
  for (int seed = 1; seed <= 50; ++seed) {
    const auto r = ekf::sim::RunScenario("urban", static_cast<std::uint64_t>(seed));
    recall_sum += r.metrics.outlier_recall;
    false_reject_sum += r.metrics.inlier_false_reject_rate;
  }
  EXPECT_GE(recall_sum / 50.0, 0.95);
  EXPECT_LE(false_reject_sum / 50.0, 0.02);
}
