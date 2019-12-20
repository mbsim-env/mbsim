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
#include "mbsim/constraints/joint_constraint.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/links/joint.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  JointConstraint::Residuum::Residuum(vector<RigidBody*> body1_, vector<RigidBody*> body2_, const Mat3xV &forceDir_, const Mat3xV &momentDir_, vector<Frame*> frame_, FrameOfReference refFrame_) : body1 (std::move(body1_)),body2(std::move(body2_)),forceDir(forceDir_),momentDir(momentDir_), frame(frame_), refFrame(refFrame_) {
  }

  Vec JointConstraint::Residuum::operator()(const Vec &x) {
    Vec res(x.size(),NONINIT); 
    int nq = 0;
    for(auto & i : body1) {
      int dq = i->getGeneralizedPositionSize();
      i->resetPositionsUpToDate();
      i->setqRel(x(nq,nq+dq-1));
      nq += dq;
    }
    for(auto & i : body2) {
      int dq = i->getGeneralizedPositionSize();
      i->resetPositionsUpToDate();
      i->setqRel(x(nq,nq+dq-1));
      nq += dq;
    }

    Mat3xV dT = frame[refFrame]->evalOrientation()*forceDir;
    Mat3xV dR = frame[refFrame]->getOrientation()*momentDir;

    if(dT.cols())
      res(Range<Var,Var>(0,dT.cols()-1)) = dT.T()*(frame[0]->evalPosition()-frame[1]->evalPosition());

    if(dR.cols())
      res(Range<Var,Var>(dT.cols(),dT.cols()+dR.cols()-1)) = dR.T()*AIK2Cardan(frame[0]->getOrientation().T()*frame[1]->getOrientation());

    return res;
  } 

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, JointConstraint)

  JointConstraint::JointConstraint(const string &name) : MechanicalConstraint(name), frame(2), C("F") {
    C.setParent(this);
  }

  void JointConstraint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for (const auto & i : saved_RigidBodyFirstSide)
        bd1.push_back(getByPath<RigidBody>(i));
      for (const auto & i : saved_RigidBodySecondSide)
        bd2.push_back(getByPath<RigidBody>(i));
      for (const auto & i : saved_IndependentBody)
        bi.push_back(getByPath<RigidBody>(i));
      if(bi.empty())
        throwError("No independent rigid bodies given!");
      if(bd1.empty() and bd2.empty())
        throwError("No dependent rigid bodies given!");
      if(not saved_ref1.empty() and not saved_ref2.empty())
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not frame[0] or not frame[1])
        throwError("Not all connections are given!");
    }
    else if(stage==preInit) {
      if(refFrame==unknown)
        throwError("(JointConstraint::init): frame of reference unknown");
      iF = RangeV(0, forceDir.cols() - 1);
      iM = RangeV(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      DF.resize(forceDir.cols(),NONINIT);
      DM.resize(momentDir.cols(),NONINIT);
      for(auto & i : bd1) 
        i->addDependency(this);
      for(auto & i : bd2)
        i->addDependency(this);
      for(auto & i : bi)
        addDependency(i);
      C.setFrameOfReference(frame[0]);
    }
    else if(stage==unknownStage) {
      C.sethSize(frame[0]->gethSize());
      C.sethSize(frame[0]->gethSize(1),1);
      C.init(stage, config);
      nq = 0;
      nu = 0;
      nh = 0;
      Iq1.resize(bd1.size());
      Iu1.resize(bd1.size());
      Ih1.resize(bd1.size());
      for(unsigned int i=0; i<bd1.size(); i++) {
        int dq = bd1[i]->getGeneralizedPositionSize();
        int du = bd1[i]->getGeneralizedVelocitySize();
        int dh = bd1[i]->gethSize(0);
        Iq1[i] = RangeV(nq,nq+dq-1);
        Iu1[i] = RangeV(nu,nu+du-1);
        Ih1[i] = RangeV(0,dh-1);
        nq += dq;
        nu += du;
        nh = max(nh,dh);
      }
      Iq2.resize(bd2.size());
      Iu2.resize(bd2.size());
      Ih2.resize(bd2.size());
      for(unsigned int i=0; i<bd2.size(); i++) {
        int dq = bd2[i]->getGeneralizedPositionSize();
        int du = bd2[i]->getGeneralizedVelocitySize();
        int dh = bd2[i]->gethSize(0);
        Iq2[i] = RangeV(nq,nq+dq-1);
        Iu2[i] = RangeV(nu,nu+du-1);
        Ih2[i] = RangeV(0,dh-1);
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
        throwError("(JointConstraint::initz): size of q0 does not match, must be " + to_string(q.size()));

      A.resize(nu);
    }
    MechanicalConstraint::init(stage, config);
  }

  void JointConstraint::resetUpToDate() {
    MechanicalConstraint::resetUpToDate();
    C.resetUpToDate();
    updDF = true;
    updA = true;
  }

  void JointConstraint::updatePositions(Frame *frame_) {
    frame_->setPosition(frame[1]->getPosition());
    frame_->setOrientation(frame[0]->evalOrientation());
  }

  void JointConstraint::updateForceDirections() {
    DF = frame[refFrame]->evalOrientation() * forceDir;
    DM = frame[refFrame]->getOrientation() * momentDir;
    updDF = false;
  }

  void JointConstraint::updateA() {
    A(iF,RangeV(0,nu-1)) = evalGlobalForceDirection().T()*JT;
    A(iM,RangeV(0,nu-1)) = getGlobalMomentDirection().T()*JR;
    updA = false;
  }

  void JointConstraint::updateGeneralizedCoordinates() {
    Residuum f(bd1,bd2,forceDir,momentDir,frame,refFrame);
    MultiDimNewtonMethod newton(&f);
    q = newton.solve(q);
    if(newton.getInfo()!=0)
      msg(Warn) << endl << "Error in JointConstraint: update of state dependent variables failed!" << endl;
    for(unsigned int i=0; i<bd1.size(); i++)
      bd1[i]->setqRel(q(Iq1[i]));
    for(unsigned int i=0; i<bd2.size(); i++)
      bd2[i]->setqRel(q(Iq2[i]));

    for(size_t i=0; i<bd1.size(); i++) {
      bd1[i]->setUpdateByReference(false);
      C.getJacobianOfTranslation(2,false).resize(bd1[i]->getGeneralizedVelocitySize(),NONINIT);
      C.getJacobianOfRotation(2,false).resize(bd1[i]->getGeneralizedVelocitySize(),NONINIT);
      JT(RangeV(0,2),Iu1[i]) = C.evalJacobianOfTranslation(2);
      JR(RangeV(0,2),Iu1[i]) = C.getJacobianOfRotation(2);
      for(size_t j=i+1; j<bd1.size(); j++)
        bd1[j]->resetJacobiansUpToDate();
      C.resetJacobiansUpToDate();
      bd1[i]->setUpdateByReference(true);
    }
    for(size_t i=0; i<bd2.size(); i++) {
      bd2[i]->setUpdateByReference(false);
      JT(RangeV(0,2),Iu2[i]) = -frame[1]->evalJacobianOfTranslation(2);
      JR(RangeV(0,2),Iu2[i]) = -frame[1]->getJacobianOfRotation(2);
      for(size_t j=i+1; j<bd2.size(); j++)
        bd2[j]->resetJacobiansUpToDate();
      bd2[i]->setUpdateByReference(true);
    }
    for(auto & i : bd1) {
      i->resetJacobiansUpToDate();
      i->setuRel(Vec(i->getGeneralizedVelocitySize()));
    }
    for(auto & i : bd2) {
      i->resetJacobiansUpToDate();
      i->setuRel(Vec(i->getGeneralizedVelocitySize()));
    }
    Vec b(nu);
    b(iF) = -(evalGlobalForceDirection().T()*(C.evalVelocity()-frame[1]->evalVelocity()));
    b(iM) = -(getGlobalMomentDirection().T()*(C.getAngularVelocity()-frame[1]->getAngularVelocity()));
    Vec u = slvLU(evalA(),b);
    for(unsigned int i=0; i<bd1.size(); i++) {
      bd1[i]->resetVelocitiesUpToDate();
      bd1[i]->setuRel(u(Iu1[i]));
    }
    for(unsigned int i=0; i<bd2.size(); i++) {
      bd2[i]->resetVelocitiesUpToDate();
      bd2[i]->setuRel(u(Iu2[i]));
    }
    C.resetVelocitiesUpToDate();
    updGC = false;
  }

  void JointConstraint::updateGeneralizedJacobians(int jj) {
    if(jj == 0) {
      for(auto & i : bd1) {
        i->setJRel(Mat(i->getGeneralizedVelocitySize(),i->gethSize()));
        i->setjRel(Vec(i->getGeneralizedVelocitySize()));
      }
      for(auto & i : bd2) {
        i->setJRel(Mat(i->getGeneralizedVelocitySize(),i->gethSize()));
        i->setjRel(Vec(i->getGeneralizedVelocitySize()));
      }
      Vec3 WvP0P1 = frame[1]->evalVelocity() - C.evalVelocity();
      Vec3 WomK0K1 = frame[1]->getAngularVelocity() - C.getAngularVelocity();

      Mat B(nu,nh);
      Mat JT0(3,nh);
      Mat JR0(3,nh);
      if(frame[0]->getJacobianOfTranslation(0,false).cols()) {
        JT0(RangeV(0,2),RangeV(0,frame[0]->getJacobianOfTranslation(0,false).cols()-1))+=C.evalJacobianOfTranslation();
        JR0(RangeV(0,2),RangeV(0,frame[0]->getJacobianOfRotation(0,false).cols()-1))+=C.getJacobianOfRotation();
      }
      if(frame[1]->getJacobianOfTranslation(0,false).cols()) {
        JT0(RangeV(0,2),RangeV(0,frame[1]->getJacobianOfTranslation(0,false).cols()-1))-=frame[1]->evalJacobianOfTranslation();
        JR0(RangeV(0,2),RangeV(0,frame[1]->getJacobianOfRotation(0,false).cols()-1))-=frame[1]->getJacobianOfRotation();
      }
      B(iF,RangeV(0,nh-1)) = -(evalGlobalForceDirection().T()*JT0);
      B(iM,RangeV(0,nh-1)) = -(getGlobalMomentDirection().T()*JR0);
      Vec b(nu);
      b(iF) = evalGlobalForceDirection().T()*(frame[1]->evalGyroscopicAccelerationOfTranslation()-C.evalGyroscopicAccelerationOfTranslation() - crossProduct(C.evalAngularVelocity(), 2.0*WvP0P1));
      b(iM) = getGlobalMomentDirection().T()*(frame[1]->getGyroscopicAccelerationOfRotation()-C.getGyroscopicAccelerationOfRotation()-crossProduct(C.getAngularVelocity(), WomK0K1));

      Mat J = slvLU(evalA(),B);
      Vec j = slvLU(getA(),b);
      for(unsigned int i=0; i<bd1.size(); i++) {
        bd1[i]->resetJacobiansUpToDate();
        bd1[i]->resetGyroscopicAccelerationsUpToDate();
        bd1[i]->setJRel(J(Iu1[i],Ih1[i]));
        bd1[i]->setjRel(j(Iu1[i]));
      }
      for(unsigned int i=0; i<bd2.size(); i++) {
       bd2[i]->resetJacobiansUpToDate();
       bd2[i]->resetGyroscopicAccelerationsUpToDate();
       bd2[i]->setJRel(J(Iu2[i],Ih2[i]));
       bd2[i]->setjRel(j(Iu2[i]));
      }
      updGJ = false;
    }
  }

  void JointConstraint::setUpInverseKinetics() {
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    if(forceDir.cols()) {
      joint->setForceDirection(forceDir);
      joint->setForceLaw(new BilateralConstraint);
    }
    if(momentDir.cols()) {
      joint->setMomentDirection(momentDir);
      joint->setMomentLaw(new BilateralConstraint);
    }
    joint->connect(frame[0],frame[1]);
    joint->plotFeature[generalizedRelativePosition] = false;
    joint->plotFeature[generalizedRelativeVelocity] = false;
    link = joint;
  }

  void JointConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    MechanicalConstraint::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodyOnFirstSide");
    while(e and E(e)->getTagName()==MBSIM%"dependentRigidBodyOnFirstSide") {
      saved_RigidBodyFirstSide.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodyOnSecondSide");
    while(e and E(e)->getTagName()==MBSIM%"dependentRigidBodyOnSecondSide") {
      saved_RigidBodySecondSide.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"independentRigidBody");
    while(e and E(e)->getTagName()==MBSIM%"independentRigidBody") {
      saved_IndependentBody.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");
    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) {
      string refFrameStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(refFrameStr=="firstFrame") refFrame=firstFrame;
      else if(refFrameStr=="secondFrame") refFrame=secondFrame;
      else refFrame=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat3xV>(3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat3xV>(3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if (e) setInitialGuess(E(e)->getText<Vec>());
  }

}
