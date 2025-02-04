/* Copyright (C) 2004-2014 MBSim Development Team
 *
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
#include "mbsim/objects/rigid_body.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include "mbsim/environment.h"
#include "mbsim/utils/xmlutils.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zyx.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zyx_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_transformed_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_transformed_mapping.h"
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Range<Var,Var> i02(0,2);

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RigidBody)

  RigidBody::RigidBody(const string &name) : Body(name),  APK(EYE),  Z("Z") {
    
    Z.setParent(this);

    C=new FixedRelativeFrame("C");
    //C->setUpdateByParent(1);
    Body::addFrame(C);
    K = C;
    openMBVFrame=C;

    updateJacobians_[0] = &RigidBody::updateJacobians0;
    updateJacobians_[1] = &RigidBody::updateJacobians1;
    updateJacobians_[2] = &RigidBody::updateJacobians2;
  }

  RigidBody::~RigidBody() {
    if(fPrPK) { delete fPrPK; fPrPK=nullptr; }
    if(fAPK) { delete fAPK; fAPK=nullptr; }
    if(fTR) { delete fTR; fTR=nullptr; }
  }

  void RigidBody::setFrameForKinematics(Frame *frame) { 
    K = dynamic_cast<FixedRelativeFrame*>(frame); 
    assert(K);
  }

  void RigidBody::setFrameForInertiaTensor(Frame *frame) { 
    frameForInertiaTensor = dynamic_cast<FixedRelativeFrame*>(frame); 
    assert(frameForInertiaTensor);
  }

  void RigidBody::updateh(int j) {
    if(j==0) {
      Vec3 WF = m*ds->getMBSimEnvironment()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(evalGlobalInertiaTensor()*C->evalAngularVelocity(),C->evalAngularVelocity()) ;
      h[j] += C->evalJacobianOfTranslation(j).T()*(WF - m*C->evalGyroscopicAccelerationOfTranslation()) + C->evalJacobianOfRotation(j).T()*(WM - getGlobalInertiaTensor()*C->evalGyroscopicAccelerationOfRotation());
    } else {
      const Vec3 &aP = Z.evalAcceleration();
      const Vec3 &Psi = Z.evalAngularAcceleration();
      const Vec3 &rPS = C->evalGlobalRelativePosition();
      const Vec3 &Om = Z.evalAngularVelocity();
      SymMat3 Theta = evalGlobalInertiaTensor() + m*JTJ(tilde(rPS));
      Vec3 WF = m*ds->getMBSimEnvironment()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(rPS,WF);
      h[j] += Z.evalJacobianOfTranslation(j).T()*(WF - m*(aP + crossProduct(Psi,rPS) + crossProduct(Om,crossProduct(Om,rPS)))) + Z.evalJacobianOfRotation(j).T()*(WM - (m*crossProduct(rPS,aP) + Theta*Psi + crossProduct(Om,Theta*Om)));
    }
  }

  void RigidBody::calcSize() {
    int nqT=0, nqR=0, nuT=0, nuR=0;
    if(fPrPK) {
      nqT = fPrPK->getArg1Size();
      nuT = fPrPK->getArg1Size(); // TODO fTT->getArg1Size()
    }
    if(fAPK) {
      nqR = fAPK->getArg1Size();
      nuR = fAPK->getArg1Size(); // TODO fTR->getArg1Size()
    }

    if(translationDependentRotation) {
      assert(nqT == nqR);
      assert(nuT == nuR);
      nq = nqT;
      nu = nuT;
      iqT = Range<Var,Var>(0,nq-1);
      iqR = Range<Var,Var>(0,nq-1);
      iuT = Range<Var,Var>(0,nu-1);
      iuR = Range<Var,Var>(0,nu-1);
    }
    else {
      nq = nqT + nqR;
      nu = nuT + nuR;
      iqT = Range<Var,Var>(0,nqT-1);
      iqR = Range<Var,Var>(nq-nqR,nq-1);
      iuT = Range<Var,Var>(0,nuT-1);
      iuR = Range<Var,Var>(nu-nuR,nu-1);
    }
    updSize = false;
  }

  void RigidBody::calcqSize() {
    Body::calcqSize();
    qSize = constraint ? 0 : nq;
  }

  void RigidBody::calcuSize(int j) {
    Body::calcuSize(j);
    if(j==0)
      uSize[j] = constraint ? 0 : nu;
    else
      uSize[j] = 6;
  }

  void RigidBody::init(InitStage stage, const InitConfigSet &config) {
    Z.init(stage, config);
    if(stage==preInit) {
      // Note we explicity make this check here to check exceptions in init
      if(m<0)
        throwError("The mass must be nonnegative.");
      if(SThetaS(0,0)<0 || SThetaS(1,1)<0 || SThetaS(2,2)<0)
        throwError("The diagonal elements of the inertia tensor must be nonnegative.");

      if(generalizedVelocityOfRotation==unknown)
        throwError("Generalized velocity of rotation unknown");

      for(unsigned int k=1; k<frame.size(); k++) {
        if(!dynamic_cast<FixedRelativeFrame*>(frame[k]))
          continue; // we allow to add none FixedRelativeFrame's to a RigidBody -> skip such frames here; its up to the caller which has added such none FixedRelativeFrame's to handle these propably
        if(not(static_cast<FixedRelativeFrame*>(frame[k])->getFrameOfReference()))
          static_cast<FixedRelativeFrame*>(frame[k])->setFrameOfReference(C);
      }

      for(auto & k : contour) {
        if(not(static_cast<RigidContour*>(k))->getFrameOfReference())
          static_cast<RigidContour*>(k)->setFrameOfReference(C);
      }

      if(K!=C) {
        const FixedRelativeFrame *R = K;
        do {
          R = static_cast<const FixedRelativeFrame*>(R->getFrameOfReference());
          K->setRelativePosition(R->getRelativePosition() + R->getRelativeOrientation()*K->getRelativePosition());
          K->setRelativeOrientation(R->getRelativeOrientation()*K->getRelativeOrientation());
        } while(R!=C);
        C->setRelativeOrientation(K->getRelativeOrientation().T());
        C->setRelativePosition(-(C->getRelativeOrientation()*K->getRelativePosition()));
        C->setFrameOfReference(K);
        K->setRelativeOrientation(SqrMat3(EYE));
        K->setRelativePosition(Vec3());
      }
      K->setFrameOfReference(&Z);

      if(constraint)
        addDependency(constraint);

      if(fPrPK) {
        qTRel.resize(fPrPK->getArg1Size(),NONINIT);
        uTRel.resize(qTRel.size(),NONINIT);
        qdTRel.resize(qTRel.size(),NONINIT);
        PJTT.resize(qTRel.size(),NONINIT);
      }
      if(fAPK) {
        qRRel.resize(fAPK->getArg1Size(),NONINIT);
        uRRel.resize(qRRel.size(),NONINIT);
        qdRRel.resize(qRRel.size(),NONINIT);
        PJRR.resize(qRRel.size(),NONINIT);
      }

      PJT[0].resize(getGeneralizedVelocitySize());
      PJR[0].resize(nu);

      PJT[1].resize(6);
      PJR[1].resize(6);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;
      jRel.resize(nu);

      frameForJacobianOfRotation = generalizedVelocityOfRotation==coordinatesOfAngularVelocityWrtFrameForKinematics?K:R;
    }
    else if(stage==unknownStage) {
      JRel[0].resize(nu,hSize[0]);
      for(int i=0; i<uSize[0]; i++)
        JRel[0](i,hSize[0]-uSize[0]+i) = 1;
      JRel[1].resize(6,hSize[1]);
      for(int i=0; i<uSize[1]; i++)
        JRel[1](i,hSize[1]-uSize[1]+i) = 1;
      T.init(Eye());

      Z.getJacobianOfTranslation(1,false) = PJT[1];
      Z.getJacobianOfRotation(1,false) = PJR[1];

      auto *Atmp = dynamic_cast<StateDependentFunction<RotMat3>*>(fAPK);
      if(Atmp and generalizedVelocityOfRotation!=derivativeOfGeneralizedPositionOfRotation) {
        RotationAboutThreeAxes<VecV> *A3 = dynamic_cast<RotationAboutThreeAxes<VecV>*>(Atmp->getFunction());
        if(A3) {
          fTR = (generalizedVelocityOfRotation==coordinatesOfAngularVelocityWrtFrameOfReference)?A3->getMappingFunction():A3->getTransformedMappingFunction();
          if(not fTR) throwError("(RigidBody::init): coordinate transformation not yet available for current rotation");
          fTR->setParent(this);
          ds->setqdequ(false);
          constJR = true;
          constjR = true;
          PJRR = SqrMat3(EYE);
          PJR[0].set(i02,iuR,PJRR);
        }
        else
          throwError("(RigidBody::init): coordinate transformation only valid for spatial rotations");
      }

      if(fPrPK) {
        if(fPrPK->constParDer1()) {
          constJT = true;
          PJTT = fPrPK->parDer1(qTRel,0);
          PJT[0].set(i02,iuT,PJTT);
        }
        if(fPrPK->constParDer2()) {
          constjT = true;
          PjhT = fPrPK->parDer2(qTRel,0);
        }
      }
      if(fAPK) {
        if(fAPK->constParDer1()) {
          constJR = true;
          PJRR = fAPK->parDer1(qRRel,0);
          PJR[0].set(i02,iuR,PJRR);
        }
        if(fAPK->constParDer2()) {
          constjR = true;
          PjhR = fAPK->parDer2(qRRel,0);
        }
      }

      if(frameForInertiaTensor and frameForInertiaTensor!=C)
        SThetaS = JMJT(C->evalOrientation().T()*frameForInertiaTensor->evalOrientation(),SThetaS) - m*JTJ(tilde(C->evalOrientation().T()*(frameForInertiaTensor->evalPosition()-C->evalPosition())));

      // the generalized mass matrix is constant, if:
      // - the rigidbody must not be part of a graph
      // - the kinematics must be defined w.r.t. frame C
      // - the Jacobians of translation and rotation must be constant
      if(parent==ds and K==C and (not fPrPK or constJT) and (not fAPK or (constJR and generalizedVelocityOfRotation!=coordinatesOfAngularVelocityWrtFrameOfReference))) {
        nonConstantMassMatrix = false;
        M = m*JTJ(PJT[0]) + JTMJ(SThetaS,PJR[0]);
        LLM = facLL(M);
      }
    }
    Body::init(stage, config);
    if(fTR) fTR->init(stage, config);
    if(fPrPK) fPrPK->init(stage, config);
    if(fAPK) fAPK->init(stage, config);
  }

  void RigidBody::setUpInverseKinetics() {
    joint = new InverseKineticsJoint(string("Joint_")+R->getParent()->getName()+"_"+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    joint->setForceDirection(Mat3xV(3,EYE));
    joint->setMomentDirection(Mat3xV(3,EYE));
    joint->setForceLaw(new BilateralConstraint);
    joint->setMomentLaw(new BilateralConstraint);
    joint->connect(R,&Z);
    joint->setBody(this);
    joint->setPlotFeature(ref(generalizedRelativePosition),false);
    joint->setPlotFeature(ref(generalizedRelativeVelocity),false);
  }

  void RigidBody::plot() {
    if(plotFeature[ref(openMBV)] and openMBVBody) {
      vector<double> data;
      data.push_back(getTime());
      Vec3 WrOS=openMBVFrame->evalPosition();
      Vec3 cardan=AIK2Cardan(openMBVFrame->getOrientation());
      data.push_back(WrOS(0));
      data.push_back(WrOS(1));
      data.push_back(WrOS(2));
      data.push_back(cardan(0));
      data.push_back(cardan(1));
      data.push_back(cardan(2));
      data.push_back(0);
      static_pointer_cast<OpenMBV::RigidBody>(openMBVBody)->append(data);
    }
    Body::plot();
  }

  void RigidBody::updateqd() {
    if(not constraint) {
      qd.set(iqT, evaluTRel());
      qd.set(iqR, fTR ? (*fTR)(evalqRRel())*getuRRel() : getuRRel());
    }
  }

  void RigidBody::updateT() {
    if(fTR) T.set(iqR,iuR, (*fTR)(evalqRRel()));
  }

  void RigidBody::updateInertiaTensor() {
    WThetaS = JTMJ(SThetaS,C->evalOrientation().T());
    updWTS = false;
  }

  void RigidBody::updateGeneralizedPositions() {
    if(constraint) {
      if(constraint->getUpdateGeneralizedCoordinates())
        constraint->updateGeneralizedCoordinates();
    }
    else
     qRel = q;
    qTRel = qRel(iqT);
    qRRel = qRel(iqR);
    updq = false;
  }

  void RigidBody::updateGeneralizedVelocities() {
    if(constraint) {
      if(constraint->getUpdateGeneralizedCoordinates())
        constraint->updateGeneralizedCoordinates();
    }
    else
     uRel = u;
    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    updu = false;
  }

  void RigidBody::updateDerivativeOfGeneralizedPositions() {
    qdTRel = evaluTRel();
    qdRRel = fTR ? (*fTR)(evalqRRel())*getuRRel() : getuRRel();
    qdRel.set(iqT,qdTRel);
    qdRel.set(iqR,qdRRel);
    updqd = false;
  }

  void RigidBody::updateGeneralizedAccelerations() {
    if(constraint)
      udRel = evalJRel()*evaludall() + evaljRel();
    else
     udRel = evalud();
    updud = false;
  }

  void RigidBody::updateGeneralizedJacobians(int j) {
    if(constraint and constraint->getUpdateGeneralizedJacobians())
      constraint->updateGeneralizedJacobians(j);
    updGJ = false;
  }

  void RigidBody::updatePositions() {
    if(fPrPK) PrPK = (*fPrPK)(evalqTRel(),getTime());
    if(fAPK) APK = (*fAPK)(evalqRRel(),getTime());
    WrPK = R->evalOrientation()*PrPK;
    updPos = false;
  }

  void RigidBody::updateVelocities() {
    if(fPrPK) WvPKrel = R->evalOrientation()*(evalPJTT()*evaluTRel() + evalPjhT());
    if(fAPK) WomPK = frameForJacobianOfRotation->evalOrientation()*(evalPJRR()*evaluRRel() + evalPjhR());
    updVel = false;
  }

  void RigidBody::updateJacobians() {
    if(fPrPK) {
      if(!constJT) {
        PJTT = fPrPK->parDer1(evalqTRel(),getTime());
        PJT[0].set(i02,iuT,PJTT);
      }
      if(!constjT)
        PjhT = fPrPK->parDer2(evalqTRel(),getTime());
    }
    if(fAPK) {
      if(!constJR) {
        PJRR = fAPK->parDer1(evalqRRel(),getTime());
        PJR[0].set(i02,iuR,PJRR);
      }
      if(!constjR)
        PjhR = fAPK->parDer2(evalqRRel(),getTime());
    }
    updPJ = false;
  }

  void RigidBody::updateGyroscopicAccelerations() {
    if(fPrPK and not(constJT and constjT))
      PjbT = (fPrPK->parDer1DirDer1(evalqdTRel(),evalqTRel(),getTime())+fPrPK->parDer1DirDer2(1,evalqTRel(),getTime()))*getuTRel() + fPrPK->parDer2DirDer1(evalqdTRel(),evalqTRel(),getTime()) + fPrPK->parDer2DirDer2(1,evalqTRel(),getTime());
    if(fAPK and not(constJR and constjR))
      PjbR = (fAPK->parDer1DirDer1(evalqdRRel(),evalqRRel(),getTime())+fAPK->parDer1DirDer2(1,evalqRRel(),getTime()))*getuRRel() + fAPK->parDer2DirDer1(evalqdRRel(),evalqRRel(),getTime()) + fAPK->parDer2DirDer2(1,evalqRRel(),getTime());
    updPjb = false;
  }

  void RigidBody::updatePositions(Frame *frame) {
    frame->setPosition(R->evalPosition() + evalGlobalRelativePosition());
    frame->setOrientation(R->getOrientation()*getRelativeOrientation());
  }

  void RigidBody::updateVelocities(Frame *frame) {
    frame->setAngularVelocity(R->evalAngularVelocity() + evalGlobalRelativeAngularVelocity());
    frame->setVelocity(R->getVelocity() + getGlobalRelativeVelocity() + crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition()));
  }

  void RigidBody::updateAccelerations(Frame *frame) {
    frame->setAcceleration(Z.evalJacobianOfTranslation()*evaludall() + Z.evalGyroscopicAccelerationOfTranslation());
    frame->setAngularAcceleration(Z.getJacobianOfRotation()*getudall() + Z.getGyroscopicAccelerationOfRotation());
  }

  void RigidBody::updateJacobians0(Frame *frame) {
    frame->getJacobianOfTranslation(0,false).init(0);
    frame->getJacobianOfRotation(0,false).init(0);
    frame->getJacobianOfTranslation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfTranslation() - tilde(evalGlobalRelativePosition())*R->evalJacobianOfRotation());
    frame->getJacobianOfRotation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfRotation());
    frame->getJacobianOfTranslation(0,false).add(i02,RangeV(0,gethSize(0)-1), R->evalOrientation()*evalPJT()*evalJRel());
    frame->getJacobianOfRotation(0,false).add(i02,RangeV(0,gethSize(0)-1), frameForJacobianOfRotation->evalOrientation()*getPJR()*getJRel());
  }

  void RigidBody::updateGyroscopicAccelerations(Frame *frame) {
    frame->setGyroscopicAccelerationOfTranslation(R->evalGyroscopicAccelerationOfTranslation() + crossProduct(R->evalGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + R->evalOrientation()*(evalPjbT() + evalPJT()*evaljRel()) + crossProduct(R->evalAngularVelocity(), 2.*evalGlobalRelativeVelocity()+crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition())));
    frame->setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->evalOrientation()*(getPjbR() + getPJR()*getjRel()) + crossProduct(R->evalAngularVelocity(), evalGlobalRelativeAngularVelocity()));
  }

  void RigidBody::updateJacobians2(Frame *frame_) {
    if(updateByReference) {
      frame_->getJacobianOfTranslation(2,false) <<= R->evalJacobianOfTranslation(2) - tilde(evalGlobalRelativePosition())*R->evalJacobianOfRotation(2);
      frame_->getJacobianOfRotation(2,false) <<= R->evalJacobianOfRotation(2);
    } else {
      frame_->getJacobianOfTranslation(2,false) <<= R->evalOrientation()*evalPJT();
      frame_->getJacobianOfRotation(2,false) <<= frameForJacobianOfRotation->evalOrientation()*getPJR();
    }
    for(auto & i : frame) {
      i->getJacobianOfTranslation(2,false).resize(frame_->getJacobianOfTranslation(2,false).cols(),NONINIT);
      i->getJacobianOfRotation(2,false).resize(frame_->getJacobianOfTranslation(2,false).cols(),NONINIT);
    }
  }

  void RigidBody::resetPositionsUpToDate() {
    Body::resetPositionsUpToDate();
    Z.resetPositionsUpToDate();
  }

  void RigidBody::resetVelocitiesUpToDate() {
    Body::resetVelocitiesUpToDate();
    Z.resetVelocitiesUpToDate();
  }

  void RigidBody::resetJacobiansUpToDate() {
    Body::resetJacobiansUpToDate();
    Z.resetJacobiansUpToDate();
  }

  void RigidBody::resetGyroscopicAccelerationsUpToDate() {
    Body::resetGyroscopicAccelerationsUpToDate();
    Z.resetGyroscopicAccelerationsUpToDate();
  }

  void RigidBody::resetUpToDate() {
    Body::resetUpToDate();
    Z.resetUpToDate();
    updPjb = true;
    updGJ = true;
    updWTS = true;
  }

  void RigidBody::setOpenMBVRigidBody(const shared_ptr<OpenMBV::RigidBody> &body) {
    openMBVBody=body;
  }

  void RigidBody::updateM() {
    M += m*JTJ(C->evalJacobianOfTranslation()) + JTMJ(evalGlobalInertiaTensor(),C->evalJacobianOfRotation());
  }

  void RigidBody::initializeUsingXML(DOMElement *element) {
    Body::initializeUsingXML(element);

    // frames
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    while(e) {
      FixedRelativeFrame *f=new FixedRelativeFrame(E(e)->getAttribute("name"));
      addFrame(f);
      f->initializeUsingXML(e);
      e=e->getNextElementSibling();
    }

    // contours
    e=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    while(e) {
      auto *c=ObjectFactory::createAndInit<RigidContour>(e);
      addContour(c);
      e=e->getNextElementSibling();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"frameForKinematics");
    if(e) setFrameForKinematics(getByPath<Frame>(E(e)->getAttribute("ref"))); // must be on of "Frame[X]" which allready exists

    e=E(element)->getFirstElementChildNamed(MBSIM%"mass");
    setMass(E(e)->getText<double>());
    // Note we explicity make this check here to check exceptions in initializeUsingXML
    if(getMass()<0) throw DOMEvalException("Mass must be nonnegative", e);

    e=E(element)->getFirstElementChildNamed(MBSIM%"inertiaTensor");
    setInertiaTensor(E(e)->getText<SymMat3>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"frameForInertiaTensor");
    if(e) setFrameForInertiaTensor(getByPath<Frame>(E(e)->getAttribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalTranslation");
    if(e) {
      auto *trans=ObjectFactory::createAndInit<Function<Vec3(VecV,double)>>(e->getFirstElementChild());
      setGeneralTranslation(trans);
    }
    else {
      e=E(element)->getFirstElementChildNamed(MBSIM%"timeDependentTranslation");
      if(e) {
        auto *trans=ObjectFactory::createAndInit<Function<Vec3(double)>>(e->getFirstElementChild());
        setTimeDependentTranslation(trans);
      }
      else {
        e=E(element)->getFirstElementChildNamed(MBSIM%"stateDependentTranslation");
        if(e) {
          auto *trans=ObjectFactory::createAndInit<Function<Vec3(VecV)>>(e->getFirstElementChild());
          setStateDependentTranslation(trans);
        }
      }
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalRotation");
    if(e) {
      auto *rot=ObjectFactory::createAndInit<Function<RotMat3(VecV,double)>>(e->getFirstElementChild());
      setGeneralRotation(rot);
    }
    else {
      e=E(element)->getFirstElementChildNamed(MBSIM%"timeDependentRotation");
      if(e) {
        auto *rot=ObjectFactory::createAndInit<Function<RotMat3(double)>>(e->getFirstElementChild());
        setTimeDependentRotation(rot);
      }
      else {
        e=E(element)->getFirstElementChildNamed(MBSIM%"stateDependentRotation");
        if(e) {
          auto *rot=ObjectFactory::createAndInit<Function<RotMat3(VecV)>>(e->getFirstElementChild());
          setStateDependentRotation(rot);
        }
      }
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"translationDependentRotation");
    if(e) translationDependentRotation = E(e)->getText<bool>();
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedVelocityOfRotation");
    if(e) {
      string generalizedVelocityOfRotationStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(generalizedVelocityOfRotationStr=="derivativeOfGeneralizedPositionOfRotation") generalizedVelocityOfRotation=derivativeOfGeneralizedPositionOfRotation;
      else if(generalizedVelocityOfRotationStr=="coordinatesOfAngularVelocityWrtFrameOfReference") generalizedVelocityOfRotation=coordinatesOfAngularVelocityWrtFrameOfReference;
      else if(generalizedVelocityOfRotationStr=="coordinatesOfAngularVelocityWrtFrameForKinematics") generalizedVelocityOfRotation=coordinatesOfAngularVelocityWrtFrameForKinematics;
      else generalizedVelocityOfRotation=unknown;
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"openMBVRigidBody");
    if(e) {
      shared_ptr<OpenMBV::RigidBody> rb=OpenMBV::ObjectFactory::create<OpenMBV::RigidBody>(e->getFirstElementChild());
      setOpenMBVRigidBody(rb);
      rb->initializeUsingXML(e->getFirstElementChild());
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"openMBVFrameOfReference");
    if(e) setOpenMBVFrameOfReference(getByPath<Frame>(E(e)->getAttribute("ref"))); // must be on of "Frame[X]" which allready exists

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVFrameC");
    if(e) {
      if(!openMBVBody) setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(e);
      C->setOpenMBVFrame(ombv.createOpenMBV());
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"plotFeatureFrameC");
    while(e and E(e)->getTagName()==MBSIM%"plotFeatureFrameC") {
      auto pf=getPlotFeatureFromXML(e);
      C->setPlotFeature(pf.first, pf.second);
      e=e->getNextElementSibling();
    }
  }

  void RigidBody::addDependency(Constraint *constraint_) {
    Body::addDependency(constraint_);
    constraint = constraint_;
  }

  void RigidBody::setqRel(const VecV &q) {
    qRel = q;
    qTRel = qRel(iqT); 
    qRRel = qRel(iqR); 
    updq = false;
  }

  void RigidBody::setuRel(const VecV &u) {
    uRel = u; 
    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    updu = false;
  }

  void RigidBody::setJRel(const MatV &J) {
    JRel[0] = J; 
    updGJ = false; 
  }

  void RigidBody::setjRel(const VecV &j) {
    jRel = j; 
    updGJ = false; 
  }

  void RigidBody::sethSize(int hSize_, int j) {
    Body::sethSize(hSize_, j);
    Z.sethSize(hSize_, j);
  }

  void RigidBody::sethInd(int hInd_, int j) {
    Body::sethInd(hInd_, j);
    Z.sethInd(hInd_, j);
  }

  void RigidBody::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Body::setDynamicSystemSolver(sys);
    if(fTR)
      fTR->setDynamicSystemSolver(sys);
    if(fPrPK)
      fPrPK->setDynamicSystemSolver(sys);
    if(fAPK)
      fAPK->setDynamicSystemSolver(sys);
  }

}
