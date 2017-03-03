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
#include "mbsim/dynamic_system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include "mbsim/environment.h"
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

  RigidBody::RigidBody(const string &name) : Body(name), m(0), coordinateTransformation(true), APK(EYE), fTR(0), fPrPK(0), fAPK(0), constraint(0), frameForJacobianOfRotation(0), frameForInertiaTensor(0), translationDependentRotation(false), constJT(false), constJR(false), constjT(false), constjR(false), updPjb(true), updGJ(true), updWTS(true), updateByReference(true), Z("Z"), bodyFixedRepresentationOfAngularVelocity(false) {
    
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
    if(fPrPK) { delete fPrPK; fPrPK=0; }
    if(fAPK) { delete fAPK; fAPK=0; }
    if(fTR) { delete fTR; fTR=0; }
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
      Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(evalGlobalInertiaTensor()*C->evalAngularVelocity(),C->evalAngularVelocity()) ;
      h[j] += C->evalJacobianOfTranslation(j).T()*(WF - m*C->evalGyroscopicAccelerationOfTranslation()) + C->evalJacobianOfRotation(j).T()*(WM - WThetaS*C->evalGyroscopicAccelerationOfRotation());
    } else {
      Vec3 aP = Z.evalAcceleration();
      Vec3 Psi = Z.evalAngularAcceleration();
      Vec3 rPS = C->evalGlobalRelativePosition();
      Vec3 Om = Z.evalAngularVelocity();
      SymMat3 Theta = evalGlobalInertiaTensor() + m*JTJ(tilde(rPS));
      Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(rPS,WF);
      h[j] += Z.evalJacobianOfTranslation(j).T()*(WF - m*(aP + crossProduct(Psi,rPS) + crossProduct(Om,crossProduct(Om,rPS)))) + Z.evalJacobianOfRotation(j).T()*(WM - (m*crossProduct(rPS,aP) + Theta*Psi + crossProduct(Om,Theta*Om)));
    }
  }

  void RigidBody::calcqSize() {
    Body::calcqSize();
    qSize = constraint ? 0 : nq;
  }

  void RigidBody::calcuSize(int j) {
    Body::calcuSize(j);
    if(j==0)
      uSize[j] = constraint ? 0 : nu[j];
    else
      uSize[j] = 6;
  }

  void RigidBody::init(InitStage stage) {
    Z.init(stage);
    if(stage==preInit) {

      for(unsigned int k=1; k<frame.size(); k++) {
        if(not(static_cast<FixedRelativeFrame*>(frame[k])->getFrameOfReference()))
          static_cast<FixedRelativeFrame*>(frame[k])->setFrameOfReference(C);
      }

      for(unsigned int k=0; k<contour.size(); k++) {
        if(not(static_cast<RigidContour*>(contour[k]))->getFrameOfReference())
          static_cast<RigidContour*>(contour[k])->setFrameOfReference(C);
      }

      Body::init(stage);

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
        nu[0] = nuT;
        iqT = Range<Var,Var>(0,nq-1);
        iqR = Range<Var,Var>(0,nq-1);
        iuT = Range<Var,Var>(0,nu[0]-1);
        iuR = Range<Var,Var>(0,nu[0]-1);
      }
      else {
        nq = nqT + nqR;
        nu[0] = nuT + nuR;
        iqT = Range<Var,Var>(0,nqT-1);
        iqR = Range<Var,Var>(nq-nqR,nq-1);
        iuT = Range<Var,Var>(0,nuT-1);
        iuR = Range<Var,Var>(nu[0]-nuR,nu[0]-1);
      }

      nu[1] = 6;

      if(constraint)
        addDependency(constraint);

      PJT[0].resize(nu[0]);
      PJR[0].resize(nu[0]);

      PJT[1].resize(nu[1]);
      PJR[1].resize(nu[1]);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;
      jRel.resize(nu[0]);
      qRel.resize(nq);
      uRel.resize(nu[0]);
      qdRel.resize(nq);
      udRel.resize(nu[0]);

      updateM_ = &RigidBody::updateMNotConst;
      updateLLM_ = &RigidBody::updateLLMNotConst;
    }
    else if(stage==unknownStage) {
      Body::init(stage);

      JRel[0].resize(nu[0],hSize[0]);
      for(int i=0; i<uSize[0]; i++)
        JRel[0](i,hSize[0]-uSize[0]+i) = 1;
      JRel[1].resize(nu[1],hSize[1]);
      for(int i=0; i<uSize[1]; i++)
        JRel[1](i,hSize[1]-uSize[1]+i) = 1;
      T.init(Eye());

      Z.getJacobianOfTranslation(1,false) = PJT[1];
      Z.getJacobianOfRotation(1,false) = PJR[1];

      StateDependentFunction<RotMat3> *Atmp = dynamic_cast<StateDependentFunction<RotMat3>*>(fAPK);
      if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesXYZ<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          fTR = new RotationAboutAxesXYZTransformedMapping<VecV>;
        else
          fTR = new RotationAboutAxesXYZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZXZ<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          fTR = new RotationAboutAxesZXZTransformedMapping<VecV>;
        else
          fTR = new RotationAboutAxesZXZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZYX<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          THROW_MBSIMERROR("(RigidBody::init): coordinate transformation not yet available for zyx-rotation");
        else
          fTR = new RotationAboutAxesZYXMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
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
          PJRR = fTR?fAPK->parDer1(qRRel,0)*(*fTR)(qRRel):fAPK->parDer1(qRRel,0);
          PJR[0].set(i02,iuR,PJRR);
        }
        if(fAPK->constParDer2()) {
          constjR = true;
          PjhR = fAPK->parDer2(qRRel,0);
        }
      }

      if(bodyFixedRepresentationOfAngularVelocity) {
        frameForJacobianOfRotation = K;
        // TODO: do not invert generalized mass matrix in case of special
        // parametrisation
//        if(K == C && dynamic_cast<DynamicSystem*>(R->getParent())) {
//          if(fPrPK) {
//            fPrPK->updateJacobian(qRel(iqT),0);
//            PJT[0].set(i02,iuT,fPrPK->getJacobian());
//          }
//          if(fAPK) {
//            fAPK->updateJacobian(qRel(iqR),0);
//            PJR[0].set(i02,iuR,fAPK->getJacobian());
//          }
//          updateM_ = &RigidBody::updateMConst;
//          Mbuf = m*JTJ(PJT[0]) + JTMJ(SThetaS,PJR[0]);
//          LLM = facLL(Mbuf);
//          facLLM_ = &RigidBody::facLLMConst;
//        }
      }
      else
        frameForJacobianOfRotation = R;

      if(frameForInertiaTensor && frameForInertiaTensor!=C)
        SThetaS = JMJT(C->evalOrientation().T()*frameForInertiaTensor->evalOrientation(),SThetaS) - m*JTJ(tilde(C->evalOrientation().T()*(frameForInertiaTensor->evalPosition()-C->evalPosition())));
    }
    else
      Body::init(stage);
    if(fTR) fTR->init(stage);
    if(fPrPK) fPrPK->init(stage);
    if(fAPK) fAPK->init(stage);
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
  }

  void RigidBody::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVBody) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 WrOS=openMBVFrame->evalPosition();
          Vec3 cardan=AIK2Cardan(openMBVFrame->evalOrientation());
          data.push_back(WrOS(0));
          data.push_back(WrOS(1));
          data.push_back(WrOS(2));
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          static_pointer_cast<OpenMBV::RigidBody>(openMBVBody)->append(data);
        }
      }
      Body::plot();
    }
  }

  void RigidBody::updateqd() {
    qd(iqT) = evaluTRel();
    if(fTR)
      qd(iqR) = (*fTR)(evalqRRel())*uRRel;
    else
      qd(iqR) = uRRel;
  }

  void RigidBody::updatedq() {
    dq(iqT) = evaluTRel()*getStepSize();
    if(fTR)
      dq(iqR) = (*fTR)(evalqRRel())*uRRel*getStepSize();
    else
      dq(iqR) = uRRel*getStepSize();
  }

  void RigidBody::updateT() {
    if(fTR) T(iqR,iuR) = (*fTR)(evalqRRel());
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
    qdRRel = fTR ? (*fTR)(evalqRRel())*uRRel : uRRel;
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
        PJRR = fTR?fAPK->parDer1(evalqRRel(),getTime())*(*fTR)(evalqRRel()):fAPK->parDer1(evalqRRel(),getTime());
        PJR[0].set(i02,iuR,PJRR);
      }
      if(!constjR)
        PjhR = fAPK->parDer2(evalqRRel(),getTime());
    }
    updPJ = false;
  }

  void RigidBody::updateGyroscopicAccelerations() {
    if(fPrPK) {
      if(not(constJT and constjT)) {
        PjbT = (fPrPK->parDer1DirDer1(evalqdTRel(),evalqTRel(),getTime())+fPrPK->parDer1ParDer2(evalqTRel(),getTime()))*uTRel + fPrPK->parDer2DirDer1(evalqdTRel(),evalqTRel(),getTime()) + fPrPK->parDer2ParDer2(evalqTRel(),getTime());
      }
    }
    if(fAPK) {
      if(not(constJR and constjR)) {
        if(fTR) {
          Mat3xV JRd = fAPK->parDer1DirDer1(evalqdRRel(),evalqRRel(),getTime())+fAPK->parDer1ParDer2(evalqRRel(),getTime());
          MatV TRd = fTR->dirDer(qdRRel,qRRel);
          PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,getTime())*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,getTime()) + fAPK->parDer2ParDer2(qRRel,getTime());
        }
        else
          PjbR = (fAPK->parDer1DirDer1(evalqdRRel(),evalqRRel(),getTime())+fAPK->parDer1ParDer2(evalqRRel(),getTime()))*uRRel + fAPK->parDer2DirDer1(evalqdRRel(),evalqRRel(),getTime()) + fAPK->parDer2ParDer2(evalqRRel(),getTime());
      }
    }
    updPjb = false;
  }

  void RigidBody::updatePositions(Frame *frame) {
    frame->setPosition(R->evalPosition() + evalGlobalRelativePosition());
    frame->setOrientation(R->getOrientation()*APK); // APK already update to date
  }

  void RigidBody::updateVelocities(Frame *frame) {
    frame->setAngularVelocity(R->evalAngularVelocity() + evalGlobalRelativeAngularVelocity());
    frame->setVelocity(R->getVelocity() + WvPKrel + crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition())); // WvPKrel already update to date
  }

  void RigidBody::updateAccelerations(Frame *frame) {
    frame->setAcceleration(Z.evalJacobianOfTranslation()*evaludall() + Z.evalGyroscopicAccelerationOfTranslation());
    frame->setAngularAcceleration(Z.getJacobianOfRotation()*udall + Z.getGyroscopicAccelerationOfRotation());
  }

  void RigidBody::updateJacobians0(Frame *frame) {
    frame->getJacobianOfTranslation(0,false).init(0);
    frame->getJacobianOfRotation(0,false).init(0);
    frame->getJacobianOfTranslation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfTranslation() - tilde(evalGlobalRelativePosition())*R->evalJacobianOfRotation());
    frame->getJacobianOfRotation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfRotation());
    frame->getJacobianOfTranslation(0,false).add(i02,RangeV(0,gethSize(0)-1), R->evalOrientation()*evalPJT()*evalJRel());
    frame->getJacobianOfRotation(0,false).add(i02,RangeV(0,gethSize(0)-1), frameForJacobianOfRotation->evalOrientation()*PJR[0]*JRel[0]);
  }

  void RigidBody::updateGyroscopicAccelerations(Frame *frame) {
    frame->setGyroscopicAccelerationOfTranslation(R->evalGyroscopicAccelerationOfTranslation() + crossProduct(R->evalGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + R->evalOrientation()*(evalPjbT() + evalPJT()*evaljRel()) + crossProduct(R->evalAngularVelocity(), 2.*evalGlobalRelativeVelocity()+crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition())));
    frame->setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->evalOrientation()*(PjbR + PJR[0]*jRel) + crossProduct(R->evalAngularVelocity(), evalGlobalRelativeAngularVelocity())); // PjbR already up to date
  }

  void RigidBody::updateJacobians2(Frame *frame_) {
    for(vector<Frame*>::iterator i=frame.begin(); i!=frame.end(); i++) {
      (*i)->getJacobianOfTranslation(2,false).resize();
      (*i)->getJacobianOfRotation(2,false).resize();
    }
    if(updateByReference) {
      frame_->getJacobianOfTranslation(2,false).resize() = R->evalJacobianOfTranslation(2) - tilde(evalGlobalRelativePosition())*R->evalJacobianOfRotation(2);
      frame_->getJacobianOfRotation(2,false).resize() = R->evalJacobianOfRotation(2);
    } else {
      frame_->getJacobianOfTranslation(2,false).resize() = R->evalOrientation()*evalPJT();
      frame_->getJacobianOfRotation(2,false).resize() = frameForJacobianOfRotation->evalOrientation()*PJR[0];
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

  void RigidBody::addFrame(FixedRelativeFrame *frame) {
    Body::addFrame(frame);
  }

  void RigidBody::addContour(RigidContour *contour) {
    Body::addContour(contour);
  }

  void RigidBody::setOpenMBVRigidBody(const shared_ptr<OpenMBV::RigidBody> &body) {
    openMBVBody=body;
  }

  void RigidBody::updateMConst() {
    M += Mbuf;
  }

  void RigidBody::updateMNotConst() {
    M += m*JTJ(C->evalJacobianOfTranslation()) + JTMJ(evalGlobalInertiaTensor(),C->evalJacobianOfRotation());
  }

  void RigidBody::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=E(element)->getFirstElementChildNamed(MBSIM%"frames")->getFirstElementChild();
    while(e) {
      FixedRelativeFrame *f=new FixedRelativeFrame(E(e)->getAttribute("name"));
      addFrame(f);
      f->initializeUsingXML(e);
      e=e->getNextElementSibling();
    }

    // contours
    e=E(element)->getFirstElementChildNamed(MBSIM%"contours")->getFirstElementChild();
    while(e) {
      RigidContour *c=ObjectFactory::createAndInit<RigidContour>(e);
      addContour(c);
      e=e->getNextElementSibling();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"frameForKinematics");
    if(e) setFrameForKinematics(getByPath<Frame>(E(e)->getAttribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=E(element)->getFirstElementChildNamed(MBSIM%"mass");
    setMass(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIM%"inertiaTensor");
    setInertiaTensor(getSymMat3(e));
    e=E(element)->getFirstElementChildNamed(MBSIM%"frameForInertiaTensor");
    if(e) setFrameForInertiaTensor(getByPath<Frame>(E(e)->getAttribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalTranslation");
    if(e && e->getFirstElementChild()) {
      Function<Vec3(VecV,double)> *trans=ObjectFactory::createAndInit<Function<Vec3(VecV,double)> >(e->getFirstElementChild());
      setGeneralTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"timeDependentTranslation");
    if(e && e->getFirstElementChild()) {
      Function<Vec3(double)> *trans=ObjectFactory::createAndInit<Function<Vec3(double)> >(e->getFirstElementChild());
      setTimeDependentTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"stateDependentTranslation");
    if(e && e->getFirstElementChild()) {
      Function<Vec3(VecV)> *trans=ObjectFactory::createAndInit<Function<Vec3(VecV)> >(e->getFirstElementChild());
      setStateDependentTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalRotation");
    if(e && e->getFirstElementChild()) {
      Function<RotMat3(VecV,double)> *rot=ObjectFactory::createAndInit<Function<RotMat3(VecV,double)> >(e->getFirstElementChild());
      setGeneralRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"timeDependentRotation");
    if(e && e->getFirstElementChild()) {
      Function<RotMat3(double)> *rot=ObjectFactory::createAndInit<Function<RotMat3(double)> >(e->getFirstElementChild());
      setTimeDependentRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"stateDependentRotation");
    if(e && e->getFirstElementChild()) {
      Function<RotMat3(VecV)> *rot=ObjectFactory::createAndInit<Function<RotMat3(VecV)> >(e->getFirstElementChild());
      setStateDependentRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"translationDependentRotation");
    if(e) translationDependentRotation = getBool(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"coordinateTransformationForRotation");
    if(e) coordinateTransformation = getBool(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"bodyFixedRepresentationOfAngularVelocity");
    if(e) bodyFixedRepresentationOfAngularVelocity = getBool(e);

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
      C->setOpenMBVFrame(ombv.createOpenMBV(e));
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"plotFeatureFrameC");
    while(e and E(e)->getTagName()==MBSIM%"plotFeatureFrameC") {
      PlotFeatureStatus status = initializePlotFeatureStatusUsingXML(e);
      PlotFeature feature = initializePlotFeatureUsingXML(e);
      C->setPlotFeature(feature, status);
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

}
