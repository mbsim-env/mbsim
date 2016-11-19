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
#include "mbsim/constraints/gear_constraint.h"
#include <mbsim/constitutive_laws/bilateral_constraint.h>
#include "mbsim/objects/rigid_body.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GearConstraint, MBSIM%"GearConstraint")

  void GearConstraint::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentRigidBody(getByPath<RigidBody>(saved_DependentBody));
      for (unsigned int i=0; i<saved_IndependentBody.size(); i++)
        bi.push_back(getByPath<RigidBody>(saved_IndependentBody[i]));
      GeneralizedConstraint::init(stage);
    }
    else if(stage==preInit) {
      GeneralizedConstraint::init(stage);
      bd->addDependency(this);
      for(unsigned int i=0; i<bi.size(); i++)
        addDependency(bi[i]);
    }
    else
      GeneralizedConstraint::init(stage);
  }

  void GearConstraint::addIndependentRigidBody(RigidBody *body, double ratio_) {
    bi.push_back(body);
    ratio.push_back(ratio_);
  }

  void GearConstraint::updateGeneralizedCoordinates() {
    bd->getqRel(false).init(0);
    bd->getuRel(false).init(0);
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getqRel(false) += bi[i]->evalqRel()*ratio[i];
      bd->getuRel(false) += bi[i]->evaluRel()*ratio[i];
    }
    updGC = false;
  }

  void GearConstraint::updateGeneralizedJacobians(int j) {
    bd->getJRel(0,false).init(0); 
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getJRel(0,false)(Range<Var,Var>(0,bi[i]->getuRelSize()-1),Range<Var,Var>(0,bi[i]->gethSize()-1)) += bi[i]->evalJRel()*ratio[i];
    }
    updGJ = false;
  }

  void GearConstraint::initializeUsingXML(DOMElement* element) {
    GeneralizedConstraint::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBody");
    saved_DependentBody=E(e)->getAttribute("ref");
    e=e->getNextElementSibling();
    while(e && E(e)->getTagName()==MBSIM%"independentRigidBody") {
      saved_IndependentBody.push_back(E(e)->getAttribute("ref"));
      ratio.push_back(stod(E(e)->getAttribute("ratio")));
      e=e->getNextElementSibling();
    }
  }

  void GearConstraint::setUpInverseKinetics() {
    Gear *gear = new Gear(string("Gear")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(gear);
    gear->setGearOutput(bd);
    for(unsigned int i=0; i<bi.size(); i++)
      gear->addGearInput(bi[i],ratio[i]);
    gear->setGeneralizedForceLaw(new BilateralConstraint);
    gear->setSupportFrame(support);
    if(FArrow)
      gear->setOpenMBVForce(FArrow);
    if(MArrow)
      gear->setOpenMBVMoment(MArrow);
  }

}
