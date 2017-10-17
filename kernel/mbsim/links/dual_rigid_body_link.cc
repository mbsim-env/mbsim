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
#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void DualRigidBodyLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveXMLPath) {
      if(saved_ref!="")
        connect(getByPath<RigidBody>(saved_ref));
      else
      if(saved_ref1!="" and saved_ref2!="")
        connect(getByPath<RigidBody>(saved_ref1),getByPath<RigidBody>(saved_ref2));
      if(not body.size())
        THROW_MBSIMERROR("No connection given!");
    }
    else if(stage==unknownStage) {
      if(body.size()>1 and (body[0]->getGeneralizedVelocitySize()!=body[1]->getGeneralizedVelocitySize()))
        THROW_MBSIMERROR("rigid bodies must have the same dof!");
    }
    RigidBodyLink::init(stage, config);
  }

  void DualRigidBodyLink::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"connect");
    if(E(e)->hasAttribute("ref")) 
      saved_ref=E(e)->getAttribute("ref");  
    else {
      saved_ref1 = E(e)->getAttribute("ref1");
      saved_ref2 = E(e)->getAttribute("ref2");
    }
  }

  void DualRigidBodyLink::connect(RigidBody *body_) {
    body.resize(1);
    ratio.resize(1);
    body[0] = body_;
    ratio[0] = 1;
  }

  void DualRigidBodyLink::connect(RigidBody *body1, RigidBody *body2) {
    body.resize(2);
    ratio.resize(2);
    body[0] = body1;
    body[1] = body2;
    ratio[0] = -1;
    ratio[1] = 1;
  }

}
