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
#include "mbsim/links/generalized_friction.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#include <mbsim/constitutive_laws/friction_force_law.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedFriction)

  GeneralizedFriction::~GeneralizedFriction() {
    delete func;
    delete laN;
  }

  void GeneralizedFriction::updateGeneralizedForces() {
    // TODO Consider set-valued friction
    lambda = (*func)(evalGeneralizedRelativeVelocity(),(*laN)(getTime()));
    updla = false;
  }

  void GeneralizedFriction::init(InitStage stage) {
    if(stage==unknownStage) {
      if(body[0]->getuRelSize()!=1)
        THROW_MBSIMERROR("rigid bodies must have 1 dof!");
    }
    DualRigidBodyLink::init(stage);
    func->init(stage);
  }

  void GeneralizedFriction::setGeneralizedFrictionForceLaw(FrictionForceLaw *func_) { 
    func = func_; 
    func->setParent(this);
  }

  void GeneralizedFriction::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedFrictionForceLaw");
    setGeneralizedFrictionForceLaw(ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedNormalForceFunction");
    setGeneralizedNormalForceFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

}
