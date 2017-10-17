/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/links/generalized_spring_damper.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedSpringDamper)

  GeneralizedSpringDamper::~GeneralizedSpringDamper() {
    delete func;
  }

  void GeneralizedSpringDamper::updateGeneralizedForces() {
    lambda(0) = -(*func)(evalGeneralizedRelativePosition()(0)-l0,evalGeneralizedRelativeVelocity()(0));
    updla = false;
  }

  void GeneralizedSpringDamper::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      if(body[0]->getGeneralizedVelocitySize()!=1)
        THROW_MBSIMERROR("rigid bodies must have 1 dof!");
    }
    DualRigidBodyLink::init(stage, config);
    func->init(stage, config);
  }

  void GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedUnloadedLength");
    l0 = Element::getDouble(e);
  }

}
