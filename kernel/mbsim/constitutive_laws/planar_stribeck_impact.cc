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
#include "mbsim/constitutive_laws/planar_stribeck_impact.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PlanarStribeckImpact, MBSIM%"PlanarStribeckImpact")

  Vec PlanarStribeckImpact::project(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    return Vec(1, INIT, proxCT2D(la(0) - r * gdn(0), (*fmu)(fabs(gdn(0))) * fabs(laN)));
  }

  Mat PlanarStribeckImpact::diff(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double r) {
    double argT = la(0) - r * gdn(0);
    Mat d(1, 3, NONINIT);
    if (fabs(argT) < (*fmu)(fabs(gdn(0))) * fabs(laN)) {
      d(0, 0) = 1;
      d(0, 1) = -r;
      d(0, 2) = 0;
    }
    else {
      d(0, 0) = 0;
      d(0, 1) = 0;
      d(0, 2) = sign(argT) * sign(laN) * (*fmu)(fabs(gdn(0)));
    }
    return d;
  }

  Vec PlanarStribeckImpact::solve(const SqrMat& G, const Vec& gdn, const Vec& gda, double laN) {
    double laNmu = fabs(laN) * (*fmu)(fabs(gdn(0)));
    double sdG = -gdn(0) / G(0, 0);
    if (fabs(sdG) <= laNmu)
      return Vec(1, INIT, sdG);
    else
      return Vec(1, INIT, (laNmu <= sdG) ? laNmu : -laNmu);
  }

  bool PlanarStribeckImpact::isFulfilled(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0) + gdn(0) / fabs(gdn(0)) * (*fmu)(fabs(gdn(0))) * fabs(laN)) <= laTol)
      return true;
    else if (fabs(la(0)) <= (*fmu)(fabs(gdn(0))) * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return true;
    else
      return false;
  }

  int PlanarStribeckImpact::isSticking(const Vec& la, const Vec& gdn, const Vec& gda, double laN, double laTol, double gdTol) {
    if (fabs(la(0)) <= (*fmu)(fabs(gdn(0))) * fabs(laN) + laTol && fabs(gdn(0)) <= gdTol)
      return 1;
    else
      return 0;
  }

  void PlanarStribeckImpact::initializeUsingXML(DOMElement *element) {
    FrictionImpactLaw::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionFunction");
    setFrictionFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

}
