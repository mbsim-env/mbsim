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
#include "mbsim/constraints/generalized_position_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/kinematic_excitation.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedPositionConstraint, MBSIM%"GeneralizedPositionConstraint")

  void GeneralizedPositionConstraint::init(InitStage stage) {
    KinematicConstraint::init(stage);
    f->init(stage);
  }

  void GeneralizedPositionConstraint::updateGeneralizedCoordinates(double t) {
    bd->getqRel(false) = (*f)(t);
    bd->getuRel(false) = f->parDer(t);
    updGC = false;
  }

  void GeneralizedPositionConstraint::updateGeneralizedJacobians(double t, int jj) {
    bd->getjRel(false) = f->parDerParDer(t);
    updGJ = false;
  }

  void GeneralizedPositionConstraint::initializeUsingXML(DOMElement* element) {
    KinematicConstraint::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"constraintFunction");
    if(e) {
      Function<VecV(double)> *f=ObjectFactory::createAndInit<Function<VecV(double)> >(e->getFirstElementChild());
      setConstraintFunction(f);
    }
  }

  void GeneralizedPositionConstraint::setUpInverseKinetics() {
    GeneralizedPositionExcitation *ke = new GeneralizedPositionExcitation(string("GeneralizedPositionExcitation")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(ke);
    ke->setDependentBody(bd);
    ke->setExcitationFunction(f);
    if(FArrow)
      ke->setOpenMBVForce(FArrow);
    if(MArrow)
      ke->setOpenMBVMoment(MArrow);
  }

}
