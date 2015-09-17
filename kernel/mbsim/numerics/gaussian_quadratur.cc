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

#include <config.h>

#include "gaussian_quadratur.h"

using namespace fmatvec;

namespace MBSim {

  void GaussPoints1D(int ngp, fmatvec::MatVx2 & xigp, fmatvec::VecV & wgp) {

    if (ngp == 1) {
      xigp(0, 0) = 0;
      wgp(0) = 2.;
    }
    else if (ngp == 2) {
      xigp(0, 0) = -1. / sqrt(3.);
      xigp(1, 0) = 1. / sqrt(3.);
      wgp(0) = 1.;
      wgp(1) = 1.;
    }
    else if (ngp == 3) {
      xigp(0, 0) = -sqrt(3. / 5.);
      xigp(1, 0) = 0;
      xigp(2, 0) = sqrt(3. / 5.);
      wgp(0) = 5. / 9.;
      wgp(1) = 8. / 9.;
      wgp(2) = 5. / 9.;
    }
    else if (ngp == 4) {
      xigp(0, 0) = -sqrt((15. + sqrt(120.)) / 35.);
      xigp(1, 0) = -sqrt((15. - sqrt(120.)) / 35.);
      xigp(2, 0) = sqrt((15. - sqrt(120.)) / 35.);
      xigp(3, 0) = sqrt((15. + sqrt(120.)) / 35.);
      wgp(0) = (18. - sqrt(30.)) / 36.;
      wgp(1) = (18. + sqrt(30.)) / 36.;
      wgp(2) = (18. + sqrt(30.)) / 36.;
      wgp(3) = (18. - sqrt(30.)) / 36.;
    }
    else
      throw MBSimError("Only up to four Gauss points implemented.");

  }

  void GaussPoints2D(int ngp, fmatvec::MatVx2 & xigp, fmatvec::VecV & wgp) {

    int ngp1D = sqrt(ngp);
    if (ngp1D % 1 != 0)
      throw MBSimError("No quadratic number of Gauss points.");

    MatVx2 xigp1D(ngp1D);
    VecV wgp1D(ngp1D);
    GaussPoints1D(ngp1D, xigp1D, wgp1D);

    int k = 0;
    for (int j = 0; j < ngp1D; j++) {
      for (int i = 0; i < ngp1D; i++) {
        xigp(k, 0) = xigp1D(i, 0);
        xigp(k, 1) = xigp1D(j, 0);
        wgp(k) = wgp1D(i) * wgp1D(j);
        k = k + 1;
      }
    }
  }

}
