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
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedConnection)

  GeneralizedConnection::GeneralizedConnection(const string &name) : RigidBodyLink(name), fl(NULL), il(NULL) {
    body[0] = NULL;
    body[1] = NULL;
  }

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
    if(stage==resolveXMLPath) {
      if(saved_body1!="")
        setRigidBodyFirstSide(getByPath<RigidBody>(saved_body1));
      if(saved_body2!="")
        setRigidBodySecondSide(getByPath<RigidBody>(saved_body2));
      if(body[1]==NULL)
        THROW_MBSIMERROR("rigid body on second side must be given!");
    }
    else if(stage==resize) {
      if(body[0]) {
        RigidBodyLink::body.resize(2);
        RigidBodyLink::body[0]=body[0];
      }
      else
        RigidBodyLink::body.resize(1);
      RigidBodyLink::body[1]=body[1];
      ratio.resize(RigidBodyLink::body.size());
      ratio[0] = -1;
      ratio[ratio.size()-1] = 1;
      RigidBodyLink::init(stage);
    }
    else if(stage==unknownStage) {
      if(fl->isSetValued()) {
        il = new BilateralImpact;
        il->setParent(this);
      }
      RigidBodyLink::init(stage);
    }
    else {
      RigidBodyLink::init(stage);
    }
    if(fl) fl->init(stage);
    if(il) il->init(stage);
  }

  void GeneralizedConnection::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceLaw");
    setGeneralizedForceLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodyFirstSide");
    if(e) saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
  }

}
