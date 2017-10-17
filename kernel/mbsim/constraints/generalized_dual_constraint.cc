/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/constraints/generalized_dual_constraint.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void GeneralizedDualConstraint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveXMLPath) {
      if(saved_DependentBody!="")
        setDependentRigidBody(getByPath<RigidBody>(saved_DependentBody));
      if(saved_IndependentBody!="")
        setIndependentRigidBody(getByPath<RigidBody>(saved_IndependentBody));
      if(not bd)
        THROW_MBSIMERROR("No dependent rigid body given!");
    }
    else if(stage==preInit) {
      bd->addDependency(this);
      if(bi) addDependency(bi);
    }
    GeneralizedConstraint::init(stage, config);
  }

  void GeneralizedDualConstraint::initializeUsingXML(DOMElement* element) {
    GeneralizedConstraint::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBody");
    saved_DependentBody=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"independentRigidBody");
    if(e) saved_IndependentBody=E(e)->getAttribute("ref");
  }

}
