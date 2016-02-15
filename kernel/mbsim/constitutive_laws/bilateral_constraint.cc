/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(BilateralConstraint, MBSIM%"BilateralConstraint")

  double BilateralConstraint::project(double la, double gdn, double r, double laMin) {
    return la - r * gdn;
  }

  Vec BilateralConstraint::diff(double la, double gdn, double r, double laMin) {
    Vec d(2, NONINIT);
    d(0) = 1;
    d(1) = -r;
    return d;
  }

  double BilateralConstraint::solve(double G, double gdn) {
    return -gdn / G;
  }

  bool BilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol, double laMin) {
    return fabs(gdn) <= gdTol;
  }

}
