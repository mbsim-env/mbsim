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
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(UnilateralConstraint, MBSIM%"UnilateralConstraint")

  double UnilateralConstraint::project(double la, double gdn, double r, double laMin) {
    return proxCN(la - r * gdn, laMin);
  }

  Vec UnilateralConstraint::diff(double la, double gdn, double r, double laMin) {
    Vec d(2, NONINIT);
    if (la - r * gdn < laMin)
      d.init(0);
    else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralConstraint::solve(double G, double gdn) {
    if (gdn >= 0)
      return 0;
    else
      return -gdn / G;
  }

  bool UnilateralConstraint::isFulfilled(double la, double gdn, double laTol, double gdTol, double laMin) {
    if (gdn >= -gdTol && fabs(la - laMin) <= laTol)
      return true;
    else if (la - laMin >= -laTol && fabs(gdn) <= gdTol)
      return true;
    else
      return false;
  }

}
