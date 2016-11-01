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
#include "mbsim/constraints/generalized_velocitiy_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/generalized_velocity_excitation.h"
#include "mbsim/dynamic_system.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedVelocityConstraint, MBSIM%"GeneralizedVelocityConstraint")

  void GeneralizedVelocityConstraint::init(InitStage stage) {
    if(stage==resize) {
      x.resize(xSize);
      KinematicConstraint::init(stage);
    }
    else
      KinematicConstraint::init(stage);
    f->init(stage);
  }

  void GeneralizedVelocityConstraint::calcxSize() {
    xSize = bd->getqRelSize();
  }

  void GeneralizedVelocityConstraint::updatexd() {
    xd = bd->transformCoordinates()?bd->evalTRel()*bd->evaluRel():bd->evaluRel();
  }

  void GeneralizedVelocityConstraint::updateGeneralizedCoordinates() {
    bd->setqRel(x);
    bd->setuRel((*f)(x,getTime()));
    updGC = false;
  }

  void GeneralizedVelocityConstraint::updateGeneralizedJacobians(int jj) {
    MatV J = f->parDer1(x,getTime());
    if(J.cols())
      bd->setjRel(J*(bd->transformCoordinates()?bd->evalTRel()*bd->evaluRel():bd->evaluRel()) + f->parDer2(x,getTime()));
    else
      bd->setjRel(f->parDer2(x,getTime()));
    updGJ = false;
  }

  void GeneralizedVelocityConstraint::initializeUsingXML(DOMElement* element) {
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

  void GeneralizedVelocityConstraint::setUpInverseKinetics() {
    GeneralizedVelocityExcitation *ke = new GeneralizedVelocityExcitation(string("GeneralizedVelocityExcitation")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(ke);
    ke->setDependentRigidBody(bd);
    ke->setExcitationFunction(f);
    ke->setSupportFrame(support);
    if(FArrow)
      ke->setOpenMBVForce(FArrow);
    if(MArrow)
      ke->setOpenMBVMoment(MArrow);
    ke->setPlotFeature(linkKinematics,disabled);
  }

}
