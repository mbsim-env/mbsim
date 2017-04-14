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
#include "mbsim/links/generalized_gear.h"
#include "mbsim/objects/rigid_body.h"
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/bilateral_impact.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedGear)

  GeneralizedGear::~GeneralizedGear() {
    delete fl;
    if(il) delete il;
  }

  bool GeneralizedGear::isSetValued() const {
    return fl->isSetValued();
  }

  void GeneralizedGear::setGeneralizedForceLaw(GeneralizedForceLaw * fl_) {
    fl=fl_;
    fl->setParent(this);
  }

  void GeneralizedGear::updateGeneralizedForces() {
    if(isSetValued())
      lambda = la;
    else
      lambda(0) = (*fl)(evalGeneralizedRelativePosition()(0),evalGeneralizedRelativeVelocity()(0));
    updla = false;
  }

  void GeneralizedGear::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_gearOutput!="")
        setGearOutput(getByPath<RigidBody>(saved_gearOutput));
      if (saved_gearInput.size()>0) {
        for (unsigned int i=0; i<saved_gearInput.size(); i++)
          body.push_back(getByPath<RigidBody>(saved_gearInput[i]));
      }
      if(not body[0])
        THROW_MBSIMERROR("No gear output given!");
      if(body.size()==1)
        THROW_MBSIMERROR("No gear inputs given!");
    }
    else if(stage==unknownStage) {
      for(unsigned int i=0; i<body.size(); i++) {
        if(body[i]->getuRelSize()!=1)
          THROW_MBSIMERROR("rigid bodies must have of 1 dof!");
      }
      if(fl->isSetValued()) {
        il = new BilateralImpact;
        il->setParent(this);
      }
    }
    RigidBodyLink::init(stage);
    if(fl) fl->init(stage);
    if(il) il->init(stage);
  }

  void GeneralizedGear::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"gearOutput");
    saved_gearOutput=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"gearOutput");
    saved_gearOutput = E(e)->getAttribute("ref");
    e=e->getNextElementSibling();
    while(e && E(e)->getTagName()==MBSIM%"gearInput") {
      saved_gearInput.push_back(E(e)->getAttribute("ref"));
      ratio.push_back(stod(E(e)->getAttribute("ratio")));
      e=e->getNextElementSibling();
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceLaw");
    setGeneralizedForceLaw(ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild()));
  }

}
