/* Copyright (C) 2004-2015 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include "lubricant.h"

#include <math.h>
#include <algorithm>
#include <mbsim/dynamic_system_solver.h>

using namespace MBSim;
using namespace std;

namespace MBSimEHD {
  Lubricant::Lubricant(const double & eta0, const double & rho0, const double & alphaOrZ, const bool & isAlpha, const ViscosityPressureModel & viscModel, const DensityPressureModel & densModel, const double & pr) :
      eta0(eta0), rho0(rho0), pr(pr) {

    if (not isAlpha) {
      Z = alphaOrZ;
      alpha = Z / cp * log(eta0 / etaInf);
    }
    else {
      alpha = alphaOrZ;
      Z = alpha * cp / log(eta0 / etaInf);
    }

    // Save used models for pressure dependence
    this->viscModel = viscModel;
    this->densModel = densModel;

    // Save reference pressure if a dimensionless description is
    // used
    if (viscModel == constVisc or viscModel == Roelands or viscModel == Barus or densModel == constDen or densModel == DowsonHigginson)
      throw MBSimError("No reference pressure for dimensionless description defined.");

  }

  void Lubricant::DynViscosity(const double & p_, double & eta, double & etadp, double & etadpdp) {

    double p = std::max(p_, 0.);

    if (viscModel == constVisc) {
      // Constant viscosity
      eta = eta0;
      etadp = 0;
      etadpdp = 0;
    }
    else if (viscModel == constViscDimLess) {
      // Constant viscosity
      eta = 1;
      etadp = 0;
      etadpdp = 0;
    }
    else if (viscModel == Roelands) {
      // Roelands formula, see Hamrock eq. (4.10)
      // Note: log = loge = ln (natural logarithm)
      eta = eta0 * pow(etaInf / eta0, 1 - pow(1 + p / cp, Z));
      etadp = -eta * Z / cp * log(etaInf / eta0) * pow(1 + p / cp, Z - 1);
      etadpdp = -etadp / cp / (1 + p / cp) * (Z * log(etaInf / eta0) * pow(1 + p / cp, Z) - (Z - 1));
    }
    else if (viscModel == RoelandsDimLess) {
      // Dimensionless Roelands formula
      double cpdl = cp / pr;
      eta = pow(etaInf / eta0, 1 - pow(1 + p / cpdl, Z));
      etadp = -eta * Z / cpdl * log(etaInf / eta0) * pow(1 + p / cpdl, Z - 1);
      etadpdp = -etadp / cpdl / (1 + p / cpdl) * (Z * log(etaInf / eta0) * pow(1 + p / cpdl, Z) - (Z - 1));

    }
    else if (viscModel == Barus) {
      // Barus formula, see Hamrock eq. (4.7)
      eta = eta0 * exp(alpha * p);
      etadp = eta0 * alpha * exp(alpha * p);
      etadpdp = eta0 * pow(alpha, 2) * exp(alpha * p);
    }
    else if (viscModel == BarusDimLess) {
      // Dimensionless Barus formula
      double alphadl = alpha * pr;
      eta = exp(alphadl * p);
      etadp = alphadl * exp(alphadl * p);
      etadpdp = pow(alphadl, 2) * exp(alphadl * p);
    }
    else {
      throw MBSimError("No valid ViscModel chosen!");
    }

  }

  void Lubricant::Density(const double & p_, double & rho, double & rhodp, double & rhodpdp) {

    double p = std::max(p_, 0.);

    if (densModel == constDen) {
      // Constant density
      rho = rho0;
      rhodp = 0;
      rhodpdp = 0;
    }
    else if (densModel == constDenDimLess) {
      // Constant density
      rho = 1;
      rhodp = 0;
      rhodpdp = 0;
    }
    else if (densModel == DowsonHigginson) {
      // Dowson and Higginson formula, see Hamrock eq. (4.19)
      rho = rho0 * (1 + a * p / (1 + b * p));
      rhodp = rho0 * a / pow(1 + b * p, 2);
      rhodpdp = -2 * rho0 * a * b / pow(1 + b * p, 3);
    }
    else if (densModel == DowsonHigginsonDimLess) {
      // Dimensionless Dowson and Higginson formula
      double adl = a * pr;
      double bdl = b * pr;
      rho = 1 + adl * p / (1 + bdl * p);
      rhodp = adl / pow(1 + bdl * p, 2);
      rhodpdp = -2 * adl * bdl / pow(1 + bdl * p, 3);
    }
    else {
      throw MBSimError("No valid densModel chosen!");
    }

  }
}
