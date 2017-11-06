/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsim/functions/kinetics/linear_regularized_stribeck_friction.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, LinearRegularizedStribeckFriction)

  Vec LinearRegularizedStribeckFriction::operator()(const Vec &gd, const double& laN) {
    int nFric = gd.size();
    Vec la(nFric, NONINIT);
    double normgd = nrm2(gd(0, nFric - 1));
    if (normgd < gdLim) {
      double mu0 = (*fmu)(0);
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu0 / gdLim);
    }
    else {
      double mu = (*fmu)(nrm2(gd(0, nFric - 1)) - gdLim);
      la(0, nFric - 1) = gd(0, nFric - 1) * (-laN * mu / normgd);
    }
    return la;
  }

  void LinearRegularizedStribeckFriction::initializeUsingXML(DOMElement *element) {
    Function<Vec(Vec,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"marginalVelocity");
    if (e)
      gdLim = E(e)->getText<double>();
    e = E(element)->getFirstElementChildNamed(MBSIM%"frictionFunction");
    Function<double(double)> *f = ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild());
    setFrictionFunction(f);
  }

}
