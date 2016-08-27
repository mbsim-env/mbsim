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
#include "mbsim/functions/kinetics/nonlinear_spring_damper_force.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NonlinearSpringDamperForce, MBSIM%"NonlinearSpringDamperForce")

  void NonlinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"forceDeflectionFunction");
    setForceDeflectionFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIM%"forceVelocityFunction");
    setForceVelocityFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

}
