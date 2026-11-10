// SPDX-License-Identifier: Apache-2.0
#include "ekf/geodesy.hpp"

#include <cmath>
#include <cstring>

namespace ekf {

double WrapPi(double angle_rad) {
  if (!std::isfinite(angle_rad)) {
    return angle_rad;
  }
  double a = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

void GeodeticToEcef(const Llh& llh, double ecef[3]) {
  const double sin_lat = std::sin(llh.lat_rad);
  const double cos_lat = std::cos(llh.lat_rad);
  const double sin_lon = std::sin(llh.lon_rad);
  const double cos_lon = std::cos(llh.lon_rad);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  ecef[0] = (n + llh.h_m) * cos_lat * cos_lon;
  ecef[1] = (n + llh.h_m) * cos_lat * sin_lon;
  ecef[2] = (n * (1.0 - kWgs84E2) + llh.h_m) * sin_lat;
}

Status EcefToGeodetic(const double ecef[3], Llh& out) {
  if (ecef[0] == 0.0 && ecef[1] == 0.0 && ecef[2] == 0.0) {
    return Status::InvalidInput;
  }
  const double x = ecef[0];
  const double y = ecef[1];
  const double z = ecef[2];
  const double p = std::sqrt(x * x + y * y);
  double lat = std::atan2(z, p * (1.0 - kWgs84E2));
  double h = 0.0;
  for (int iter = 0; iter < 10; ++iter) {
    const double sin_lat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    h = p / std::cos(lat) - n;
    const double lat_new = std::atan2(z, p * (1.0 - kWgs84E2 * n / (n + h)));
    if (std::abs(lat_new - lat) < 1e-14) {
      lat = lat_new;
      break;
    }
    lat = lat_new;
  }
  const double sin_lat = std::sin(lat);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  h = p / std::cos(lat) - n;
  out.lat_rad = lat;
  out.lon_rad = std::atan2(y, x);
  out.h_m = h;
  return Status::Ok;
}

Status MakeOrigin(const Llh& llh, Origin& origin) {
  if (!std::isfinite(llh.lat_rad) || !std::isfinite(llh.lon_rad) || !std::isfinite(llh.h_m)) {
    return Status::InvalidInput;
  }
  origin.llh = llh;
  GeodeticToEcef(llh, origin.ecef);

  const double sin_lat = std::sin(llh.lat_rad);
  const double cos_lat = std::cos(llh.lat_rad);
  const double sin_lon = std::sin(llh.lon_rad);
  const double cos_lon = std::cos(llh.lon_rad);

  origin.R_enu_from_ecef[0] = -sin_lon;
  origin.R_enu_from_ecef[1] = cos_lon;
  origin.R_enu_from_ecef[2] = 0.0;
  origin.R_enu_from_ecef[3] = -sin_lat * cos_lon;
  origin.R_enu_from_ecef[4] = -sin_lat * sin_lon;
  origin.R_enu_from_ecef[5] = cos_lat;
  origin.R_enu_from_ecef[6] = cos_lat * cos_lon;
  origin.R_enu_from_ecef[7] = cos_lat * sin_lon;
  origin.R_enu_from_ecef[8] = sin_lat;
  return Status::Ok;
}

void EcefToEnu(const Origin& origin, const double ecef[3], Enu& enu) {
  const double dx[3] = {ecef[0] - origin.ecef[0], ecef[1] - origin.ecef[1],
                        ecef[2] - origin.ecef[2]};
  enu.e = origin.R_enu_from_ecef[0] * dx[0] + origin.R_enu_from_ecef[1] * dx[1] +
          origin.R_enu_from_ecef[2] * dx[2];
  enu.n = origin.R_enu_from_ecef[3] * dx[0] + origin.R_enu_from_ecef[4] * dx[1] +
          origin.R_enu_from_ecef[5] * dx[2];
  enu.u = origin.R_enu_from_ecef[6] * dx[0] + origin.R_enu_from_ecef[7] * dx[1] +
          origin.R_enu_from_ecef[8] * dx[2];
}

void EnuToEcef(const Origin& origin, const Enu& enu, double ecef[3]) {
  const double Rt[9] = {origin.R_enu_from_ecef[0], origin.R_enu_from_ecef[3],
                        origin.R_enu_from_ecef[6], origin.R_enu_from_ecef[1],
                        origin.R_enu_from_ecef[4], origin.R_enu_from_ecef[7],
                        origin.R_enu_from_ecef[2], origin.R_enu_from_ecef[5],
                        origin.R_enu_from_ecef[8]};
  ecef[0] = origin.ecef[0] + Rt[0] * enu.e + Rt[1] * enu.n + Rt[2] * enu.u;
  ecef[1] = origin.ecef[1] + Rt[3] * enu.e + Rt[4] * enu.n + Rt[5] * enu.u;
  ecef[2] = origin.ecef[2] + Rt[6] * enu.e + Rt[7] * enu.n + Rt[8] * enu.u;
}

}  // namespace ekf
