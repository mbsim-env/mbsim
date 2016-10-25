/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "mbsim/links/gear.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Gear, MBSIM%"Gear")

  Gear::Gear(const string &name) : RigidBodyLink(name), func(0) {
    connect(NULL, -1);
  }

  void Gear::addTransmission(const Transmission &transmission) { 
    connect(transmission.body, transmission.ratio);
  }

  void Gear::updateGeneralizedForces() {
    if(func)
      lambda(0) = -(*func)(evalGeneralizedRelativePosition()(0),evalGeneralizedRelativeVelocity()(0));
    else
      lambda = la;
    updla = false;
  }

 void Gear::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentBody(getByPath<RigidBody>(saved_DependentBody));
      if (saved_IndependentBody.size()>0) {
        for (unsigned int i=0; i<saved_IndependentBody.size(); i++)
          body.push_back(getByPath<RigidBody>(saved_IndependentBody[i]));
      }
      RigidBodyLink::init(stage);
    }
    else if(stage==unknownStage) {
      for(int i=0; i<body.size(); i++) {
        if(body[i] and body[i]->getuRelSize()!=1)
          THROW_MBSIMERROR("rigid bodies must have of 1 dof!");
      }
      RigidBodyLink::init(stage);
    }
    else {
      RigidBodyLink::init(stage);
    }
    if(func) func->init(stage);
  }

  void Gear::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    if(e) {
      Function<double(double,double)> *f=ObjectFactory::createAndInit<Function<double(double,double)> >(e->getFirstElementChild());
      setGeneralizedForceFunction(f);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBody");
    saved_DependentBody=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"transmissions");
    DOMElement *ee=e->getFirstElementChild();
    while(ee && E(ee)->getTagName()==MBSIM%"Transmission") {
      saved_IndependentBody.push_back(E(E(ee)->getFirstElementChildNamed(MBSIM%"rigidBody"))->getAttribute("ref"));
      ratio.push_back(getDouble(E(ee)->getFirstElementChildNamed(MBSIM%"ratio")));
      ee=ee->getNextElementSibling();
    }
  }

}
