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
#include "mbsim/constitutive_laws/spatial_stribeck_friction.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpatialStribeckFriction)

  Vec SpatialStribeckFriction::project(const Vec& la, const Vec& gdn, double laN, double r) {
    return proxCT3D(la - r * gdn, (*fmu)(nrm2(gdn)) * fabs(laN));
  }

  Mat SpatialStribeckFriction::diff(const Vec& la, const Vec& gdn, double laN, double r) {
    Vec argT = la - r * gdn;
    Mat E(2, 2, EYE);
    Mat d(2, 5, NONINIT);
    if (nrm2(argT) < (*fmu)(nrm2(gdn)) * fabs(laN)) {
      d(Index(0, 1), Index(0, 1)) = E;
      d(Index(0, 1), Index(2, 3)) = -r * E;
      d(Index(0, 1), Index(4, 4)).init(0);
    }
    else {
      Mat d_dargT = (E - (argT * argT.T()) / (argT.T() * argT)) * (*fmu)(nrm2(gdn)) * la(0) / nrm2(argT);
      d(Index(0, 1), Index(0, 1)) = d_dargT;
      d(Index(0, 1), Index(2, 3)) = -r * d_dargT;
      d(Index(0, 1), Index(4, 4)) = argT / nrm2(argT) * (*fmu)(nrm2(gdn));
    }
    return d;
  }

  Vec SpatialStribeckFriction::solve(const SqrMat& G, const Vec& gdn, double laN) {
    throw MBSimError("(SpatialStribeckFriction::solve): Not implemented!");
  }

  bool SpatialStribeckFriction::isFulfilled(const Vec& la, const Vec& gdn, double laN, double laTol, double gdTol) {
    if (nrm2(la + gdn / nrm2(gdn) * (*fmu)(nrm2(gdn)) * fabs(laN)) <= laTol)
      return true;
    else if (nrm2(la) <= (*fmu)(nrm2(gdn)) * fabs(laN) + laTol && nrm2(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  Vec SpatialStribeckFriction::dlaTdlaN(const Vec& gd) {
    return -(*fmu)(nrm2(gd)) * gd / nrm2(gd);
  }

  void SpatialStribeckFriction::initializeUsingXML(DOMElement *element) {
    FrictionForceLaw::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionFunction");
    setFrictionFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

}
