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
#include "mbsim/utils/utils.h"
#include "mbsim/objectfactory.h"
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>

#include <utility>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  JointConstraint::Residuum::Residuum(vector<RigidBody*> body1_, vector<RigidBody*> body2_, const Mat3xV &forceDir_, const Mat3xV &momentDir_,Frame *frame1_, Frame *frame2_, Frame *refFrame_, vector<Frame*> i1_, vector<Frame*> i2_) : body1 (std::move(body1_)),body2(std::move(body2_)),forceDir(forceDir_),momentDir(momentDir_),frame1(frame1_), frame2(frame2_), refFrame(refFrame_), i1(std::move(i1_)), i2(std::move(i2_)) {
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

    Mat3xV dT = refFrame->evalOrientation()*forceDir;
    Mat3xV dR = refFrame->getOrientation()*momentDir;

    if(dT.cols())
      res(Range<Var,Var>(0,dT.cols()-1)) = dT.T()*(frame1->evalPosition()-frame2->evalPosition());

    if(dR.cols())
      res(Range<Var,Var>(dT.cols(),dT.cols()+dR.cols()-1)) = dR.T()*AIK2Cardan(frame1->getOrientation().T()*frame2->getOrientation());

    return res;
  } 

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, JointConstraint)

  JointConstraint::JointConstraint(const string &name) : MechanicalConstraint(name),  refFrame(nullptr),  C("F"),  saved_ref1(""), saved_ref2("") {
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
      if((bd1.empty()) and (bd2.empty()))
        throwError("No dependent rigid bodies given!");
      if(!saved_ref1.empty() && !saved_ref2.empty())
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(frame1==nullptr or frame2==nullptr)
        throwError("Not all connections are given!");
      if(!bd1.empty()) {
        for(unsigned int i=0; i<bd1.size()-1; i++) 
          if1.push_back(bd1[i+1]->getFrameOfReference());
        if1.push_back(frame1);
      }
      if(!bd2.empty()) {
        for(unsigned int i=0; i<bd2.size()-1; i++) 
          if2.push_back(bd2[i+1]->getFrameOfReference());
        if2.push_back(frame2);
      }
    }
    else if(stage==preInit) {
      iF = RangeV(0, forceDir.cols() - 1);
      iM = RangeV(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      for(auto & i : bd1) 
        i->addDependency(this);
      for(auto & i : bd2)
        i->addDependency(this);
      for(auto & i : bi)
        addDependency(i);
      refFrame=refFrameID?frame2:frame1;
      C.setFrameOfReference(frame1);
    }
    else if(stage==unknownStage) {
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

      Js.resize(3 - forceDir.cols());
      if (forceDir.cols() == 2)
        Js.set(0, crossProduct(forceDir.col(0), forceDir.col(1)));
      else if (forceDir.cols() == 3)
        ;
      else if (forceDir.cols() == 0)
        Js = SqrMat(3, EYE);
      else { // define a coordinate system in the plane perpendicular to the force direction
        Js.set(0, computeTangential(forceDir.col(0)));
        Js.set(1, crossProduct(forceDir.col(0), Js.col(0)));
      }
      if(q0() == nullptr)
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

  void JointConstraint::updatePositions(Frame *frame) {
    frame->setPosition(frame2->getPosition());
    frame->setOrientation(frame1->evalOrientation());
  }

  void JointConstraint::updateForceDirections() {
    DF = refFrame->evalOrientation() * forceDir;
    DM = refFrame->getOrientation() * momentDir;
    updDF = false;
  }

  void JointConstraint::updateA() {
    A(iF,RangeV(0,nu-1)) = evalGlobalForceDirection().T()*JT;
    A(iM,RangeV(0,nu-1)) = getGlobalMomentDirection().T()*JR;
    updA = false;
  }

  void JointConstraint::updateGeneralizedCoordinates() {
    Residuum f(bd1,bd2,forceDir,momentDir,frame1,frame2,refFrame,if1,if2);
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
      JT(RangeV(0,2),Iu1[i]) = C.evalJacobianOfTranslation(2);
      JR(RangeV(0,2),Iu1[i]) = C.getJacobianOfRotation(2);
      for(size_t j=i+1; j<bd1.size(); j++)
        bd1[j]->resetJacobiansUpToDate();
      C.resetJacobiansUpToDate();
      bd1[i]->setUpdateByReference(true);
    }
    for(size_t i=0; i<bd2.size(); i++) {
      bd2[i]->setUpdateByReference(false);
      JT(RangeV(0,2),Iu2[i]) = -frame2->evalJacobianOfTranslation(2);
      JR(RangeV(0,2),Iu2[i]) = -frame2->getJacobianOfRotation(2);
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
    b(iF) = -(evalGlobalForceDirection().T()*(C.evalVelocity()-frame2->evalVelocity()));
    b(iM) = -(getGlobalMomentDirection().T()*(C.getAngularVelocity()-frame2->getAngularVelocity()));
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
      Vec3 WvP0P1 = frame2->evalVelocity() - C.evalVelocity();
      Vec3 WomK0K1 = frame2->getAngularVelocity() - C.getAngularVelocity();
      Mat3xV WJs = refFrame->evalOrientation() * Js;
      VecV sdT = WJs.T() * WvP0P1;

      Mat B(nu,nh);
      Mat JT0(3,nh);
      Mat JR0(3,nh);
      if(frame1->getJacobianOfTranslation(0,false).cols()) {
        JT0(RangeV(0,2),RangeV(0,frame1->getJacobianOfTranslation(0,false).cols()-1))+=C.evalJacobianOfTranslation();
        JR0(RangeV(0,2),RangeV(0,frame1->getJacobianOfRotation(0,false).cols()-1))+=C.getJacobianOfRotation();
      }
      if(frame2->getJacobianOfTranslation(0,false).cols()) {
        JT0(RangeV(0,2),RangeV(0,frame2->getJacobianOfTranslation(0,false).cols()-1))-=frame2->evalJacobianOfTranslation();
        JR0(RangeV(0,2),RangeV(0,frame2->getJacobianOfRotation(0,false).cols()-1))-=frame2->getJacobianOfRotation();
      }
      B(iF,RangeV(0,nh-1)) = -(evalGlobalForceDirection().T()*JT0);
      B(iM,RangeV(0,nh-1)) = -(getGlobalMomentDirection().T()*JR0);
      Vec b(nu);
      b(iF) = evalGlobalForceDirection().T()*(frame2->evalGyroscopicAccelerationOfTranslation()-C.evalGyroscopicAccelerationOfTranslation() - crossProduct(C.evalAngularVelocity(), WvP0P1 + WJs * sdT));
      b(iM) = getGlobalMomentDirection().T()*(frame2->getGyroscopicAccelerationOfRotation()-C.getGyroscopicAccelerationOfRotation()-crossProduct(C.getAngularVelocity(), WomK0K1));

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
    joint->connect(frame1,frame2);
    joint->plotFeature[generalizedRelativePosition] = false;
    joint->plotFeature[generalizedRelativeVelocity] = false;
    link = joint;
  }

  void JointConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    MechanicalConstraint::initializeUsingXML(element);
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodyOnFirstSide");
    while(e && E(e)->getTagName()==MBSIM%"dependentRigidBodyOnFirstSide") {
      saved_RigidBodyFirstSide.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"dependentRigidBodyOnSecondSide");
    while(e && E(e)->getTagName()==MBSIM%"dependentRigidBodyOnSecondSide") {
      saved_RigidBodySecondSide.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"independentRigidBody");
    while(e && E(e)->getTagName()==MBSIM%"independentRigidBody") {
      saved_IndependentBody.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");
    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if(e) refFrameID=E(e)->getText<int>()-1;
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat3xV>(3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat3xV>(3));
    e=E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if (e) setInitialGuess(E(e)->getText<Vec>());
  }

  void JointConstraint::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void JointConstraint::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }

}
