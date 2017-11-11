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
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"
#include "mbsim/objectfactory.h"
#include "mbsim/utils/nonsmooth_algebra.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, UnilateralNewtonImpact)

  double UnilateralNewtonImpact::project(double la, double gdn, double gda, double r, double laMin) {
    if (gda <= -gd_limit) {       // 2 Aenderungen :
      gdn += epsilon * gda;       // elastischer Anteil nur bei negativer Annäherungsgeschw. ueber gd_limit
    }                           // zwischen gd_limit und gd_limit/10 wird eps stetig auf 0 zurueckgefuehrt
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }
    return proxCN(la - r * gdn, laMin);
  }

  Vec UnilateralNewtonImpact::diff(double la, double gdn, double gda, double r, double laMin) {
    Vec d(2, NONINIT);
    if (la - laMin - r * gdn < 0)
      d.init(0);
    else {
      d(0) = 1;
      d(1) = -r;
    }
    return d;
  }

  double UnilateralNewtonImpact::solve(double G, double gdn, double gda) {
    if (gda <= -gd_limit) {
      gdn += epsilon * gda;
    }
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }

    if (gdn >= 0)
      return 0;
    else
      return -gdn / G;
  }

  bool UnilateralNewtonImpact::isFulfilled(double la, double gdn, double gda, double laTol, double gdTol, double laMin) {
    if (gda <= -gd_limit) {
      gdn += epsilon * gda;
    }
    else {
      if (gda < -0.1 * gd_limit) {
        double epsi = epsilon * 0.5 * (cos(M_PI / (0.9 * gd_limit) * (fabs(gda) - gd_limit)) + 1);
        gdn += epsi * gda;
      }
    }
    if (gdn >= -gdTol && fabs(la - laMin) <= laTol)
      return true;
    else if (la - laMin >= -laTol && fabs(gdn) <= gdTol)
      return true;
    else
      return false;
  }

  void UnilateralNewtonImpact::initializeUsingXML(DOMElement *element) {
    GeneralizedImpactLaw::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"restitutionCoefficient");
    epsilon = E(e)->getText<double>();
  }

}
