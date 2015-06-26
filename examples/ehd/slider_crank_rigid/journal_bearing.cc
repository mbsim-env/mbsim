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
 * Created on: Jul 30, 2013
 * Contact: kilian.grundl@gmail.com
 */

#include "journal_bearing.h"

#include <mbsim/mbsim_event.h>

using namespace fmatvec;
using namespace MBSim;

namespace MBSimEHD {

  JournalBearing::JournalBearing(bool dimLess) :
      dimLess(dimLess) {
    // Compute radius of journal
    R1 = R2 - h0;

    // Build mass matrices of both bodies
    M1(0, 0) = m;
    M1(1, 1) = m;
    M1(2, 2) = J;

    M2(0, 0) = m;
    M2(1, 1) = m;
    M2(2, 2) = J;

    // Set characteristic size of fluid domain and film thickness
    // for dimensionless description
    if (dimLess) {
      xrF = L;
      hrF = h0;
    }

  }

  void JournalBearing::Thickness(const fmatvec::VecV & x, const int & e, const int & g, double & h1, double & h2, double & h1dy, double & h2dy) const {

    double y = x(1);
    if (dimLess) {
      y = y * xrF;
    }

    // Retrieve eccentricity in coordinate em F
    double er, et, r1;
    Eccentricity(y, er, et, r1);

    // Compute distances h1 and h2
    h1 = er + r1;
    h2 = R2;

    // Compute derivatives of h1 and h2 with respect to y
    h1dy = et * h1 / (R2 * r1);
    h2dy = 0;

    // Check if the film thickness is smaller than zero, meaning
    // if there is a penetration of the contacting bodies
    if (h2 - h1 < 0)
      throw MBSimError("Film thickness is smaller than zero, i.e. h < 0.");

    if (dimLess) {
      h1 = h1 / hrF;
      h2 = h2 / hrF;
      h1dy = h1dy * xrF / hrF;
      h2dy = h2dy * xrF / hrF;
    }
  }

  void JournalBearing::Velocities(const fmatvec::VecV & x, const int & e, const int & g, double & u1, double & u2, double & v1, double & v2, double & v1dy, double & v2dy) const {

    VecV y(1);
    y(0) = x(1);
    if (dimLess) {
      y = y * xrF;
    }

    double phi;
    SqrMat2 AFK;
    AngleCoordSys(y(0), phi, AFK);

    double er, et, r1;
    Eccentricity(y(0), er, et, r1);

    double h2;
    double void1;
    Thickness(y, void1, void1, void1, h2, void1, void1);

    // Compute derivative of second line from rotation matrix AFK
    RowVec2 AFKdphi2;
    AFKdphi2(0) = -cos(phi);
    AFKdphi2(1) = -sin(phi);

    // Compute velocities on journal surface
    // Note: K coincides with I for fixed bearing shell (phi2 = 0)
    u1 = AFK.row(0) * IuS1 + omega1 * et;
    v1 = AFK.row(1) * IuS1 + omega1 * r1;

    // Compute velocities on inner bearing shell surface
    u2 = AFK.row(0) * IuS2;
    v2 = AFK.row(1) * IuS2 + omega2 * h2;

    // Compute derivative of tangential velocities
    v1dy = 1 / R2 * (AFKdphi2 * IuS1 + omega1 * et * er / r1);
    v2dy = 1 / R2 * AFKdphi2 * IuS2;

    if (dimLess) {
      v1dy = v1dy * xrF;
      v2dy = v2dy * xrF;
    }
  }

  void JournalBearing::Eccentricity(const double & y, double & er, double & et, double & r1) const {
// Get rotation matrix
    double phi;
    SqrMat2 AFK;
    AngleCoordSys(y, phi, AFK);

// Compute eccentricity in coordinate system K
// Note: K coincides with I for fixed bearing shell (phi2 = 0)
    Vec2 Ke(IxS1 - IxS2);

// Transform eccentricity into coordinate system F
    er = AFK.row(0) * Ke;
    et = AFK.row(1) * Ke;

// Compute auxiliary length variable
    r1 = sqrt(pow(R1, 2) - pow(et, 2));
  }

  void JournalBearing::AngleCoordSys(const double & y, double & phi, fmatvec::SqrMat2 & AFK) const {

// Define mapping
    phi = y / R2;

// Build rotation matrix
    AFK(0, 0) = cos(phi);
    AFK(0, 1) = sin(phi);
    AFK(1, 0) = -sin(phi);
    AFK(1, 1) = cos(phi);
  }

}
