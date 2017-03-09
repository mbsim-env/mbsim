/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/links/generalized_connection.h"
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/bilateral_impact.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedConnection)

  GeneralizedConnection::~GeneralizedConnection() {
    delete fl;
    if(il) delete il;
  }

  bool GeneralizedConnection::isSetValued() const {
    return fl->isSetValued();
  }

  void GeneralizedConnection::setGeneralizedForceLaw(GeneralizedForceLaw * fl_) {
    fl=fl_;
    fl->setParent(this);
  }

  void GeneralizedConnection::updateGeneralizedForces() {
    if(isSetValued())
      lambda = la;
    else
      lambda(0) = (*fl)(evalGeneralizedRelativePosition()(0),evalGeneralizedRelativeVelocity()(0));
    updla = false;
  }

  void GeneralizedConnection::init(InitStage stage) {
    if(stage==unknownStage) {
      if(fl->isSetValued()) {
        il = new BilateralImpact;
        il->setParent(this);
      }
    }
    DualRigidBodyLink::init(stage);
    if(fl) fl->init(stage);
    if(il) il->init(stage);
  }

  void GeneralizedConnection::initializeUsingXML(DOMElement* element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceLaw");
    setGeneralizedForceLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
  }

}
