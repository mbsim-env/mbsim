/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "mbsim/constraints/kinematic_constraint.h"
#include "mbsim/objects/rigid_body.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {
 
  KinematicConstraint::KinematicConstraint(const std::string &name) : GeneralizedConstraint(name), bd(0), saved_DependentBody("") {
  }

  void KinematicConstraint::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentRigidBody(getByPath<RigidBody>(saved_DependentBody));
      GeneralizedConstraint::init(stage);
    }
    else if(stage==preInit) {
      GeneralizedConstraint::init(stage);
      bd->addDependency(this);
    }
    else
      GeneralizedConstraint::init(stage);
  }

  void KinematicConstraint::initializeUsingXML(DOMElement* element) {
    GeneralizedConstraint::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBody");
    saved_DependentBody=E(e)->getAttribute("ref");

#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FArrow=ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      MArrow=ombv.createOpenMBV(e);
    }
#endif
  }

}
