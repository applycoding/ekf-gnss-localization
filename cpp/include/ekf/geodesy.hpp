// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ekf/types.hpp"

namespace ekf {

double WrapPi(double angle_rad);

Status MakeOrigin(const Llh& llh, Origin& origin);
void GeodeticToEcef(const Llh& llh, double ecef[3]);
Status EcefToGeodetic(const double ecef[3], Llh& out);
void EcefToEnu(const Origin& origin, const double ecef[3], Enu& enu);
void EnuToEcef(const Origin& origin, const Enu& enu, double ecef[3]);

}  // namespace ekf
