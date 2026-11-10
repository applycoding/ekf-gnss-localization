// SPDX-License-Identifier: Apache-2.0
#include "ekf/geodesy.hpp"
#include "ekf_sim/rng.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

constexpr double kDeg2Rad = ekf::kPi / 180.0;

void expect_ecef_near(const double got[3], double ex, double ey, double ez, double tol_m) {
  EXPECT_NEAR(got[0], ex, tol_m);
  EXPECT_NEAR(got[1], ey, tol_m);
  EXPECT_NEAR(got[2], ez, tol_m);
}

}  // namespace

TEST(Geodesy, AnchorPoints) {
  double ecef[3]{};
  ekf::GeodeticToEcef({0.0, 0.0, 0.0}, ecef);
  expect_ecef_near(ecef, 6378137.0, 0.0, 0.0, 1e-4);

  ekf::GeodeticToEcef({0.0, 90.0 * kDeg2Rad, 0.0}, ecef);
  expect_ecef_near(ecef, 0.0, 6378137.0, 0.0, 1e-4);

  ekf::GeodeticToEcef({90.0 * kDeg2Rad, 0.0, 0.0}, ecef);
  expect_ecef_near(ecef, 0.0, 0.0, 6356752.314245179, 1e-4);
  ekf::GeodeticToEcef({-90.0 * kDeg2Rad, 0.0, 0.0}, ecef);
  expect_ecef_near(ecef, 0.0, 0.0, -6356752.314245179, 1e-4);

  ekf::GeodeticToEcef({35.6812 * kDeg2Rad, 139.7671 * kDeg2Rad, 40.0}, ecef);
  expect_ecef_near(ecef, -3959690.802569, 3350097.500459, 3699540.124670, 1e-4);

  ekf::GeodeticToEcef({45.0 * kDeg2Rad, 45.0 * kDeg2Rad, 1000.0}, ecef);
  expect_ecef_near(ecef, 3194919.145061, 3194919.145061, 4488055.515647, 1e-4);
}

TEST(Geodesy, EcefZeroInvalid) {
  ekf::Llh out{};
  const double z[3] = {0.0, 0.0, 0.0};
  EXPECT_EQ(ekf::EcefToGeodetic(z, out), ekf::Status::InvalidInput);
}

TEST(Geodesy, EnuBasisAtEquator) {
  ekf::Origin origin{};
  ASSERT_EQ(ekf::MakeOrigin({0.0, 0.0, 0.0}, origin), ekf::Status::Ok);
  double ecef_e[3]{};
  double ecef_n[3]{};
  double ecef_u[3]{};
  ekf::EnuToEcef(origin, {1.0, 0.0, 0.0}, ecef_e);
  ekf::EnuToEcef(origin, {0.0, 1.0, 0.0}, ecef_n);
  ekf::EnuToEcef(origin, {0.0, 0.0, 1.0}, ecef_u);
  EXPECT_NEAR(ecef_e[0] - origin.ecef[0], 0.0, 1e-4);
  EXPECT_NEAR(ecef_e[1] - origin.ecef[1], 1.0, 1e-4);
  EXPECT_NEAR(ecef_e[2] - origin.ecef[2], 0.0, 1e-4);
  EXPECT_NEAR(ecef_n[0] - origin.ecef[0], 0.0, 1e-4);
  EXPECT_NEAR(ecef_n[1] - origin.ecef[1], 0.0, 1e-4);
  EXPECT_NEAR(ecef_n[2] - origin.ecef[2], 1.0, 1e-4);
  EXPECT_NEAR(ecef_u[0] - origin.ecef[0], 1.0, 1e-4);
  EXPECT_NEAR(ecef_u[1] - origin.ecef[1], 0.0, 1e-4);
  EXPECT_NEAR(ecef_u[2] - origin.ecef[2], 0.0, 1e-4);
}

TEST(Geodesy, SeededRoundTrips) {
  ekf::sim::Rng rng(12345U);
  for (int i = 0; i < 1000; ++i) {
    const double lat =
        (rng.uniform(ekf::sim::RngStream::Gyro, 0.0, 1.0) * 178.0 - 89.0) * kDeg2Rad;
    const double lon = rng.uniform(ekf::sim::RngStream::Accel, 0.0, 1.0) * 2.0 * ekf::kPi -
                       ekf::kPi;
  const double h = rng.uniform(ekf::sim::RngStream::Wheel, 0.0, 1.0) * 9500.0 - 500.0;
    const ekf::Llh in{lat, lon, h};
    double ecef[3];
    ekf::GeodeticToEcef(in, ecef);
    ekf::Llh back{};
    ASSERT_EQ(ekf::EcefToGeodetic(ecef, back), ekf::Status::Ok);
    double ecef2[3];
    ekf::GeodeticToEcef(back, ecef2);
    EXPECT_NEAR(ecef[0], ecef2[0], 1e-6);
    EXPECT_NEAR(ecef[1], ecef2[1], 1e-6);
    EXPECT_NEAR(ecef[2], ecef2[2], 1e-6);
    EXPECT_NEAR(back.lat_rad, in.lat_rad, 1e-12);
    EXPECT_NEAR(std::remainder(back.lon_rad - in.lon_rad, 2.0 * ekf::kPi), 0.0, 1e-12);
  }
}
