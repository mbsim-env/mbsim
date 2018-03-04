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
#include "mbsim/constitutive_laws/spatial_coulomb_friction.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpatialCoulombFriction)

  Vec SpatialCoulombFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return proxCT3D(la - r * gdn, mu * fabs(laN));
  }

  Mat SpatialCoulombFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < mu * fabs(laN)) {
      //d_dargT = Mat(2,2,EYE);
      d(RangeV(0, 1), RangeV(0, 1)) = E;
      d(RangeV(0, 1), RangeV(2, 3)) = -r * E;
      d(RangeV(0, 1), RangeV(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * mu * la(0) / nrm2(argT);
      d(RangeV(0, 1), RangeV(0, 1)) = d_dargT;
      d(RangeV(0, 1), RangeV(2, 3)) = -r * d_dargT;
      d(RangeV(0, 1), RangeV(4, 4)) = argT / nrm2(argT) * mu;
    }
    return d;
  }

  Vec SpatialCoulombFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    throwError("(SpatialCoulombFriction::solve): Not implemented!");
  }

  bool SpatialCoulombFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * mu * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= mu * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  Vec SpatialCoulombFriction::dlaTdlaN(const Vec& gd) {
    return -mu * gd / nrm2(gd);
  }

  void SpatialCoulombFriction::initializeUsingXML(DOMElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionCoefficient");
    setFrictionCoefficient(E(e)->getText<double>());
  }

}
