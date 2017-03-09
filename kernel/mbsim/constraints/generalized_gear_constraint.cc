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
#include "mbsim/constraints/generalized_gear_constraint.h"
#include "mbsim/links/generalized_gear.h"
#include <mbsim/constitutive_laws/bilateral_constraint.h>
#include "mbsim/objects/rigid_body.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedGearConstraint)

  void GeneralizedGearConstraint::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if (saved_DependentBody!="")
        setDependentRigidBody(getByPath<RigidBody>(saved_DependentBody));
      for (unsigned int i=0; i<saved_IndependentBody.size(); i++)
        bi.push_back(getByPath<RigidBody>(saved_IndependentBody[i]));
    }
    else if(stage==preInit) {
      bd->addDependency(this);
      for(unsigned int i=0; i<bi.size(); i++)
        addDependency(bi[i]);
    }
    GeneralizedConstraint::init(stage);
  }

  void GeneralizedGearConstraint::addIndependentRigidBody(RigidBody *body, double ratio_) {
    bi.push_back(body);
    ratio.push_back(ratio_);
  }

  void GeneralizedGearConstraint::updateGeneralizedCoordinates() {
    bd->getGeneralizedPosition(false).init(0);
    bd->getGeneralizedVelocity(false).init(0);
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getGeneralizedPosition(false) += bi[i]->evalGeneralizedPosition()*ratio[i];
      bd->getGeneralizedVelocity(false) += bi[i]->evalGeneralizedVelocity()*ratio[i];
    }
    updGC = false;
  }

  void GeneralizedGearConstraint::updateGeneralizedJacobians(int j) {
    bd->getJRel(0,false).init(0); 
    for(unsigned int i=0; i<bi.size(); i++) {
      bd->getJRel(0,false).add(Range<Var,Var>(0,bi[i]->getuRelSize()-1),Range<Var,Var>(0,bi[i]->gethSize()-1),bi[i]->evalJRel()*ratio[i]);
    }
    updGJ = false;
  }

  void GeneralizedGearConstraint::initializeUsingXML(DOMElement* element) {
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

  void GeneralizedGearConstraint::setUpInverseKinetics() {
    GeneralizedGear *gear = new GeneralizedGear(string("GeneralizedGear")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(gear);
    gear->setGearOutput(bd);
    for(unsigned int i=0; i<bi.size(); i++)
      gear->addGearInput(bi[i],ratio[i]);
    gear->setGeneralizedForceLaw(new BilateralConstraint);
    gear->setSupportFrame(support);
    link = gear;
  }

}
