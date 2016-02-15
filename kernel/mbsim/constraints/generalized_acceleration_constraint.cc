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
#include "mbsim/constraints/generalized_acceleration_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/kinematic_excitation.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedAccelerationConstraint, MBSIM%"GeneralizedAccelerationConstraint")

  void GeneralizedAccelerationConstraint::init(InitStage stage) {
    if(stage==resize) {
      x.resize(xSize);
      KinematicConstraint::init(stage);
    }
    else
      KinematicConstraint::init(stage);
    f->init(stage);
  }

  void GeneralizedAccelerationConstraint::calcxSize() {
    xSize = bd->getqRelSize()+bd->getuRelSize();
  }

  void GeneralizedAccelerationConstraint::updatexd(double t) {
    xd(0,bd->getqRelSize()-1) = bd->transformCoordinates()?bd->getTRel(t)*bd->getuRel(t):bd->getuRel(t);
    xd(bd->getqRelSize(),bd->getqRelSize()+bd->getuRelSize()-1) = bd->getjRel(t);
  }

  void GeneralizedAccelerationConstraint::updateGeneralizedCoordinates(double t) {
    bd->getqRel(false) = x(0,bd->getqRelSize()-1);
    bd->getuRel(false) = x(bd->getqRelSize(),bd->getqRelSize()+bd->getuRelSize()-1);
    updGC = false;
  }

  void GeneralizedAccelerationConstraint::updateGeneralizedJacobians(double t, int jj) {
    bd->getjRel(false) = (*f)(x,t);
    updGJ = false;
  }

  void GeneralizedAccelerationConstraint::initializeUsingXML(DOMElement* element) {
    KinematicConstraint::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"initialState");
    if(e)
      x0 = getVec(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalConstraintFunction");
    if(e) {
      Function<VecV(VecV,double)> *f=ObjectFactory::createAndInit<Function<VecV(VecV,double)> >(e->getFirstElementChild());
      setGeneralConstraintFunction(f);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"timeDependentConstraintFunction");
    if(e) {
      Function<VecV(double)> *f=ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild());
      setTimeDependentConstraintFunction(f);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"stateDependentConstraintFunction");
    if(e) {
      Function<VecV(VecV)> *f=ObjectFactory::createAndInit<Function<VecV(VecV)> >(e->getFirstElementChild());
      setStateDependentConstraintFunction(f);
    }
  }

  void GeneralizedAccelerationConstraint::setUpInverseKinetics() {
    GeneralizedAccelerationExcitation *ke = new GeneralizedAccelerationExcitation(string("GeneralizedAccelerationExcitation")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(ke);
    ke->setDependentBody(bd);
    ke->setExcitationFunction(f);
    if(FArrow)
      ke->setOpenMBVForce(FArrow);
    if(MArrow)
      ke->setOpenMBVMoment(MArrow);
  }

}
