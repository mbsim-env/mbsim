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
#include "mbsim/functions/linear_spring_damper_force.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinearSpringDamperForce, MBSIM%"LinearSpringDamperForce")

  LinearSpringDamperForce::LinearSpringDamperForce(double c_, double d_, double l0_) : c(c_), d(d_), l0(l0_) { 
    Deprecated::registerMessage("The parameter unloadedLength in class LinearSpringDamperForce is deprecated. Use the paramter unloadedLength of class SpringDamper instead."); 
  }

  void LinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    Function<double(double,double)>::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"stiffnessCoefficient");
    c = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"dampingCoefficient");
    d = Element::getDouble(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"unloadedLength");
    if(e) {
      l0 = Element::getDouble(e);
      Deprecated::registerMessage("The parameter unloadedLength in class LinearSpringDamperForce is deprecated. Use the paramter unloadedLength of class SpringDamper instead.");
    }
  }

}
