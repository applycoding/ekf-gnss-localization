// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/types.hpp"

namespace ekf::sim {

constexpr double kDtS = 0.01;
constexpr int kNumTicks = 12500;
constexpr double kDurationS = 125.0;

constexpr double kOriginLatRad = 35.6812 * ekf::kPi / 180.0;
constexpr double kOriginLonRad = 139.7671 * ekf::kPi / 180.0;
constexpr double kOriginHeightM = 40.0;

constexpr double kGyroMeasSigmaRadS = 0.02;
constexpr double kAccelMeasSigmaMS2 = 0.5;
constexpr double kWheelMeasSigmaMS = 0.05;
constexpr double kGnssWhiteSigmaM = 1.5;
constexpr double kGnssBiasSigmaM = 1.0;
constexpr double kGnssBiasTauS = 60.0;
constexpr double kDefaultGyroBiasRadS = 0.01;

constexpr double kMultipathProb = 0.05;
constexpr double kMultipathMinM = 10.0;
constexpr double kMultipathMaxM = 30.0;

constexpr double kDropoutStartS = 50.0;
constexpr double kDropoutEndS = 75.0;

ekf::Config DefaultFilterConfig();

}  // namespace ekf::sim
