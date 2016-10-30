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
#include "mbsim/objects/rigid_body.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedConnection, MBSIM%"GeneralizedConnection")

  GeneralizedConnection::GeneralizedConnection(const string &name) : RigidBodyLink(name), func(0) {
    body[0] = NULL;
    body[1] = NULL;
  }

  GeneralizedConnection::~GeneralizedConnection() {
    if(func) delete func;
  }

  void GeneralizedConnection::updateGeneralizedForces() {
    if(func)
      lambda(0) = -(*func)(evalGeneralizedRelativePosition()(0),evalGeneralizedRelativeVelocity()(0));
    else
      lambda = la;
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
    else {
      RigidBodyLink::init(stage);
    }
    if(func) func->init(stage);
  }

  void GeneralizedConnection::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodyFirstSide");
    if(e) saved_body1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBodySecondSide");
    saved_body2=E(e)->getAttribute("ref");
  }

}
