/* Copyright (C) 2004-2012 MBSim Development Team
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

#include "newton_method_jacobian_functions.h"

#include <mbsim/utils/eps.h>

using namespace fmatvec;

namespace MBSim {

  NewtonJacobianFunction::NewtonJacobianFunction() :
    function(0){
  }

  NumericalNewtonJacobianFunction::NumericalNewtonJacobianFunction() :
    NewtonJacobianFunction() {
  }

  SqrMat NumericalNewtonJacobianFunction::operator ()(const fmatvec::Vec & x, const void*) {
    SqrMat J = SqrMat(x.size()); // initialize size

    double dx, xj;
    Vec x2 = x;
    Vec f = (*function)(x2);
    Vec f2;

    for (int j = 0; j < x2.size(); j++) {
      xj = x2(j);

      dx = (epsroot() * 0.5);
      do {
        dx += dx;
      } while (fabs(xj + dx - x2(j)) < epsroot());

      x2(j) += dx;
      f2 = (*function)(x2);
      x2(j) = xj;
      J.col(j) = (f2 - f) / dx;
    }

    return J;
  }

}
