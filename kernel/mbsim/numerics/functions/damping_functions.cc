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

#include <config.h>
#include "damping_functions.h"

using namespace fmatvec;


namespace MBSim {

  DampingFunction::DampingFunction()  {

  }

  StandardDampingFunction::StandardDampingFunction(unsigned int kmax_ , double d_) :
      DampingFunction(), kmax(kmax_), d(d_) {
  }

  double StandardDampingFunction::operator ()(const Vec &x, const fmatvec::Vec &dx) {
    double alpha = 1;
    Vec xnew = x;

    for (unsigned int k = 0; k < kmax; k++) {
      xnew = x - alpha * dx;
//      Vec f = (*function)(xnew);
      if(criteria->isBetter(xnew)) {
        return alpha;
      }
      alpha *= d;
    }

    return 1;
  }

}
