/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "mbsim/constraints/inverse_kinematics_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/links/generalized_kinematic_excitation.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/objectfactory.h"
#include "mbsim/functions/constant_function.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  InverseKinematicsConstraint::Residuum::Residuum(const Vec3 &r_, const RotMat3 &A_, vector<RigidBody*> body_, const Mat3xV &forceDir_, const Mat3xV &momentDir_, Frame* frame_) : r(r_), A(A_), body(std::move(body_)),forceDir(forceDir_),momentDir(momentDir_), frame(frame_) {
  }

  Vec InverseKinematicsConstraint::Residuum::operator()(const Vec &x) {
    Vec res(x.size(),NONINIT); 
    int nq = 0;
    for(auto & i : body) {
      int dq = i->getGeneralizedPositionSize();
      i->resetPositionsUpToDate();
      i->setqRel(x(nq,nq+dq-1));
      nq += dq;
    }

    if(forceDir.cols())
      res(Range<Var,Var>(0,forceDir.cols()-1)) = forceDir.T()*(frame->evalPosition()-r);

    if(momentDir.cols())
      res(Range<Var,Var>(forceDir.cols(),forceDir.cols()+momentDir.cols()-1)) = momentDir.T()*AIK2Cardan(frame->getOrientation().T()*A);

    return res;
  } 

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, InverseKinematicsConstraint)

  InverseKinematicsConstraint::~InverseKinematicsConstraint() {
    if(fr) delete fr;
    if(fA) delete fA;
  }

  void InverseKinematicsConstraint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(kinematics==unknown)
        throwError("(InverseKinematicsConstraint::init): kinematics unknown!");
      if(not saved_ref.empty())
        frame = getByPath<Frame>(saved_ref);
      if(not frame)
        throwError("Frame must be given!");
      if(kinematics==spatial) {
        forceDir = Mat3xV(3,Eye());
        if(fA) {
          bd.resize(6);
          momentDir = Mat3xV(3,Eye());
        }
        else
          bd.resize(3);
      }
      else {
        forceDir = Mat3xV("[1,0;0,1;0,0]");
        if(fA) {
          bd.resize(3);
          momentDir = Mat3xV("[0;0;1]");
        }
        else
          bd.resize(2);
      }
      vector<RigidBody*> tmp(bd.size());
      RigidBody *body = dynamic_cast<RigidBody*>(frame->getParent());
      for(size_t i=0; i<bd.size(); i++) {
        if(not body)
          throwError("Body must be given!");
        tmp[i] = body;
        body = dynamic_cast<RigidBody*>(body->getFrameOfReference()->getParent());
      }
      for(size_t i=0; i<bd.size(); i++)
        bd[i] = tmp[bd.size()-i-1];
      link.resize(bd.size());
    }
    else if(stage==preInit) {
      iF = RangeV(0, forceDir.cols() - 1);
      iM = RangeV(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      for(auto & i : bd) 
        i->addDependency(this);
    }
    else if(stage==unknownStage) {
      nq = 0;
      nu = 0;
      nh = 0;
      Iq.resize(bd.size());
      Iu.resize(bd.size());
      Ih.resize(bd.size());
      for(unsigned int i=0; i<bd.size(); i++) {
        int dq = bd[i]->getGeneralizedPositionSize();
        int du = bd[i]->getGeneralizedVelocitySize();
        int dh = bd[i]->gethSize(0);
        Iq[i] = RangeV(nq,nq+dq-1);
        Iu[i] = RangeV(nu,nu+du-1);
        Ih[i] = RangeV(0,dh-1);
        nq += dq;
        nu += du;
        nh = max(nh,dh);
      }

      q.resize(nq);
      JT.resize(3,nu);
      JR.resize(3,nu);

      if(not q0())
        q.init(0);
      else if(q0.size() == q.size())
        q = q0;
      else
        throwError("(InverseKinematicsConstraint::initz): size of q0 does not match, must be " + to_string(q.size()));

      A.resize(nu);
    }
    if(fr) fr->init(stage, config);
    if(fA) fA->init(stage, config);
    Constraint::init(stage, config);
  }

  void InverseKinematicsConstraint::resetUpToDate() {
    Constraint::resetUpToDate();
    updA = true;
  }

  void InverseKinematicsConstraint::updateA() {
    A(iF,RangeV(0,nu-1)) = forceDir.T()*JT;
    A(iM,RangeV(0,nu-1)) = momentDir.T()*JR;
    updA = false;
  }

  void InverseKinematicsConstraint::updateGeneralizedCoordinates() {
    Residuum f((*fr)(getTime()),fA?(*fA)(getTime()):RotMat3(Eye()),bd,forceDir,momentDir,frame);
    MultiDimNewtonMethod newton(&f);
    q = newton.solve(q);
    if(newton.getInfo()!=0)
      msg(Warn) << endl << "Error in InverseKinematicsConstraint: update of state dependent variables failed!" << endl;
    for(unsigned int i=0; i<bd.size(); i++)
      bd[i]->setqRel(q(Iq[i]));

    for(size_t i=0; i<bd.size(); i++) {
      bd[i]->setUpdateByReference(false);
      JT(RangeV(0,2),Iu[i]) = frame->evalJacobianOfTranslation(2);
      JR(RangeV(0,2),Iu[i]) = frame->getJacobianOfRotation(2);
      for(size_t j=i+1; j<bd.size(); j++)
        bd[j]->resetJacobiansUpToDate();
      frame->resetJacobiansUpToDate();
      bd[i]->setUpdateByReference(true);
    }
    for(auto & i : bd) {
      i->resetJacobiansUpToDate();
      i->setuRel(Vec(i->getGeneralizedVelocitySize()));
    }
    Vec b(nu);
    b(iF) = -(forceDir.T()*(frame->evalVelocity()-fr->parDer(getTime())));
    if(fA) b(iM) = -(momentDir.T()*(frame->getAngularVelocity()-fA->parDer(getTime())));
    Vec u = slvLU(evalA(),b);
    for(unsigned int i=0; i<bd.size(); i++) {
      bd[i]->resetVelocitiesUpToDate();
      bd[i]->setuRel(u(Iu[i]));
    }
    frame->resetVelocitiesUpToDate();
    updGC = false;
  }

  void InverseKinematicsConstraint::updateGeneralizedJacobians(int jj) {
    if(jj == 0) {
      for(auto & i : bd) {
        i->setJRel(Mat(i->getGeneralizedVelocitySize(),i->gethSize()));
        i->setjRel(Vec(i->getGeneralizedVelocitySize()));
      }
      Vec b(nu);
      Vec3 WvP0P1 = fr->parDer(getTime()) - frame->evalVelocity();
      b(iF) = forceDir.T()*(fr->parDerParDer(getTime())-frame->evalGyroscopicAccelerationOfTranslation() - crossProduct(frame->evalAngularVelocity(), 2.0*WvP0P1));
      if(fA) {
        Vec3 WomK0K1 = fA->parDer(getTime()) - frame->getAngularVelocity();
        b(iM) = momentDir.T()*(fA->parDerParDer(getTime())-frame->getGyroscopicAccelerationOfRotation()-crossProduct(frame->getAngularVelocity(), WomK0K1));
      }

      Vec j = slvLU(evalA(),b);
      for(unsigned int i=0; i<bd.size(); i++) {
        bd[i]->resetJacobiansUpToDate();
        bd[i]->resetGyroscopicAccelerationsUpToDate();
        bd[i]->setjRel(j(Iu[i]));
      }
      updGJ = false;
    }
  }

  void InverseKinematicsConstraint::setUpInverseKinetics() {
    for(size_t i=0; i<bd.size(); i++) {
      GeneralizedKinematicExcitation *ke = new GeneralizedKinematicExcitation(string("GeneralizedKinematicExcitation_")+name+"_"+bd[i]->getName());
      static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(ke);
      ke->connect(bd[i]);
      ke->setGeneralizedForceLaw(new BilateralConstraint);
      ke->setPlotFeature(generalizedRelativePosition, false);
      ke->setPlotFeature(generalizedRelativeVelocity, false);
      link[i] = ke;
    }
  }

  void InverseKinematicsConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Constraint::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIM%"kinematics");
    if(e) {
      string kinematicsStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(kinematicsStr=="planar") kinematics=planar;
      else if(kinematicsStr=="spatial") kinematics=spatial;
      else kinematics=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"frame");
    saved_ref=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"translation");
    setTranslation(ObjectFactory::createAndInit<Function<Vec3(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"rotation");
    if(e) setRotation(ObjectFactory::createAndInit<Function<RotMat3(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if(e) setInitialGuess(E(e)->getText<Vec>());
  }

}
