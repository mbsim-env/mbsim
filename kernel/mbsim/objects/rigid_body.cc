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
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_transformed.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_transformed_mapping.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  Range<Var,Var> i02(0,2);

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RigidBody, MBSIM%"RigidBody")

  RigidBody::RigidBody(const string &name) : Body(name), m(0), coordinateTransformation(true), APK(EYE), fTR(0), fPrPK(0), fAPK(0), constraint(0), frameForJacobianOfRotation(0), frameForInertiaTensor(0), translationDependentRotation(false), constJT(false), constJR(false), constjT(false), constjR(false), updPjb(true), updGC(true), updGJ(true), updWTS(true), updT(true), updateByReference(true), Z("Z") {
    
    Z.setParent(this);

    C=new FixedRelativeFrame("C");
    //C->setUpdateByParent(1);
    Body::addFrame(C);
    K = C;
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=C;
#endif

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

  void RigidBody::updateh(double t, int j) {
    if(j==0) {
      Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(getGlobalInertiaTensor(t)*C->getAngularVelocity(t),C->getAngularVelocity(t)) ;
      h[j] += C->getJacobianOfTranslation(t,j).T()*(WF - m*C->getGyroscopicAccelerationOfTranslation(t)) + C->getJacobianOfRotation(t,j).T()*(WM - WThetaS*C->getGyroscopicAccelerationOfRotation(t));
    } else {
      Vec3 aP = Z.getAcceleration(t);
      Vec3 Psi = Z.getAngularAcceleration(t);
      Vec3 rPS = C->getGlobalRelativePosition(t);
      Vec3 Om = Z.getAngularVelocity(t);
      SymMat3 Theta = getGlobalInertiaTensor(t) + m*JTJ(tilde(rPS));
      Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      Vec3 WM = crossProduct(rPS,WF);
      h[j] += Z.getJacobianOfTranslation(t,j).T()*(WF - m*(aP + crossProduct(Psi,rPS) + crossProduct(Om,crossProduct(Om,rPS)))) + Z.getJacobianOfRotation(t,j).T()*(WM - (m*crossProduct(rPS,aP) + Theta*Psi + crossProduct(Om,Theta*Om)));
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
    }
    else if(stage==relativeFrameContourLocation) {
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
    }
    else if(stage==resize) {
      Body::init(stage);

      PJT[0].resize(nu[0]);
      PJR[0].resize(nu[0]);

      PJT[1].resize(nu[1]);
      PJR[1].resize(nu[1]);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;

      JRel[0].resize(nu[0],hSize[0]);
      for(int i=0; i<uSize[0]; i++)
        JRel[0](i,hSize[0]-uSize[0]+i) = 1;
      JRel[1].resize(nu[1],hSize[1]);
      for(int i=0; i<uSize[1]; i++)
        JRel[1](i,hSize[1]-uSize[1]+i) = 1;
      jRel.resize(nu[0]);
      qRel.resize(nq);
      uRel.resize(nu[0]);
      q.resize(qSize);
      u.resize(uSize[0]);

      TRel.resize(nq,nu[0],Eye());

      updateM_ = &RigidBody::updateMNotConst;
      updateLLM_ = &RigidBody::updateLLMNotConst;
    }
    else if(stage==unknownStage) {
      Body::init(stage);

      Z.getJacobianOfTranslation(1,false) = PJT[1];
      Z.getJacobianOfRotation(1,false) = PJR[1];

      bool cb = false;
      StateDependentFunction<RotMat3> *Atmp = dynamic_cast<StateDependentFunction<RotMat3>*>(fAPK);
      if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesXYZ<VecV>*>(Atmp->getFunction())) {
        fTR = new RotationAboutAxesXYZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and dynamic_cast<RotationAboutAxesXYZTransformed<VecV>*>(Atmp->getFunction())) {
        cb = true;
        if(coordinateTransformation) {
          fTR = new RotationAboutAxesXYZTransformedMapping<VecV>;
          fTR->setParent(this);
          constJR = true;
          constjR = true;
          PJRR = SqrMat3(EYE);
          PJR[0].set(i02,iuR,PJRR);
        }
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZXZ<VecV>*>(Atmp->getFunction())) {
        fTR = new RotationAboutAxesZXZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZYX<VecV>*>(Atmp->getFunction())) {
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

      if(cb) {
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
//          LLM[0] = facLL(Mbuf);
//          facLLM_ = &RigidBody::facLLMConst;
//        }
      }
      else
        frameForJacobianOfRotation = R;

      T.init(Eye());

      if(frameForInertiaTensor && frameForInertiaTensor!=C)
        SThetaS = JMJT(C->getOrientation(0.).T()*frameForInertiaTensor->getOrientation(0.),SThetaS) - m*JTJ(tilde(C->getOrientation(0.).T()*(frameForInertiaTensor->getPosition(0.)-C->getPosition(0.))));
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(notMinimalState)==enabled) {
          for(int i=0; i<nq; i++)
            plotColumns.push_back("qRel("+numtostr(i)+")");
          for(int i=0; i<nu[0]; i++)
            plotColumns.push_back("uRel("+numtostr(i)+")");
        }
        Body::init(stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(getPlotFeature(openMBV)==enabled && FWeight) {
            FWeight->setName("Weight");
            openMBVGrp->addObject(FWeight);
          }
        }
#endif
      }
    }
    else
      Body::init(stage);
    if(fTR) fTR->init(stage);
    if(fPrPK) fPrPK->init(stage);
    if(fAPK) fAPK->init(stage);
    Z.init(stage);
  }

  void RigidBody::initz() {
    Body::initz();
    if(!constraint) { 
      qRel>>q;
      uRel>>u;
      TRel>>T;
    }
  }

  void RigidBody::setUpInverseKinetics() {
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+R->getParent()->getName()+"_"+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    joint->setForceDirection(Mat3xV(3,EYE));
    joint->setMomentDirection(Mat3xV(3,EYE));
    joint->setForceLaw(new BilateralConstraint);
    joint->setMomentLaw(new BilateralConstraint);
    joint->connect(R,&Z);
    joint->setBody(this);
    if(FArrow)
      joint->setOpenMBVForce(FArrow);
    if(MArrow)
      joint->setOpenMBVMoment(MArrow);
  }

  void RigidBody::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(notMinimalState)==enabled) {
        for(int i=0; i<nq; i++)
          plotVector.push_back(qRel(i));
        for(int i=0; i<nu[0]; i++)
          plotVector.push_back(uRel(i));
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(FWeight) {
          vector<double> data;
          data.push_back(t);
          Vec3 WrOS=C->getPosition(t);
          Vec3 WG = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
          data.push_back(WrOS(0));
          data.push_back(WrOS(1));
          data.push_back(WrOS(2));
          data.push_back(WG(0));
          data.push_back(WG(1));
          data.push_back(WG(2));
          data.push_back(1.0);
          FWeight->append(data);
        }
        if(openMBVBody) {
          vector<double> data;
          data.push_back(t);
          Vec3 WrOS=openMBVFrame->getPosition(t);
          Vec3 cardan=AIK2Cardan(openMBVFrame->getOrientation(t));
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
#endif
      Body::plot(t,dt);
    }
  }

  void RigidBody::updateqd(double t) {
    if(!constraint) {
      qd(iqT) = getuTRel(t);
      if(fTR)
        qd(iqR) = (*fTR)(qRRel)*uRRel;
      else
        qd(iqR) = uRRel;
    }
  }

  void RigidBody::updatedq(double t, double dt) {
    if(!constraint) {
      qd(iqT) = getuTRel(t)*dt;
      if(fTR)
        qd(iqR) = (*fTR)(qRRel)*uRRel*dt;
      else
        qd(iqR) = uRRel*dt;
    }
  }

  void RigidBody::updateT(double t) {
    if(fTR) TRel(iqR,iuR) = (*fTR)(getqRRel(t));
    updT = false;
  }

  void RigidBody::updateInertiaTensor(double t) {
    WThetaS = JTMJ(SThetaS,C->getOrientation(t).T());
    updWTS = false;
  }

  void RigidBody::updateGeneralizedCoordinates(double t) {
    if(constraint and constraint->updateGeneralizedCoordinates()) constraint->updateGeneralizedCoordinates(t);
    qTRel = qRel(iqT);
    qRRel = qRel(iqR);
    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    updGC = false;
  }

  void RigidBody::updateGeneralizedJacobians(double t, int j) {
    if(constraint and constraint->updateGeneralizedJacobians()) constraint->updateGeneralizedJacobians(t,j);
    updGJ = false;
  }

  void RigidBody::updatePositions(double t) {
    if(fPrPK) PrPK = (*fPrPK)(getqTRel(t),t);
    if(fAPK) APK = (*fAPK)(getqRRel(t),t);
    WrPK = R->getOrientation(t)*PrPK;
    updPos = false;
  }

  void RigidBody::updateVelocities(double t) {
    if(fPrPK) WvPKrel = R->getOrientation(t)*(getPJTT(t)*getuTRel(t) + PjhT);
    if(fAPK) WomPK = frameForJacobianOfRotation->getOrientation(t)*(getPJRR(t)*getuRRel(t) + PjhR);
    updVel = false;
  }

  void RigidBody::updateJacobians(double t) {
    if(fPrPK) {
      if(!constJT) {
        PJTT = fPrPK->parDer1(getqTRel(t),t);
        PJT[0].set(i02,iuT,PJTT);
      }
      if(!constjT)
        PjhT = fPrPK->parDer2(getqTRel(t),t);
    }
    if(fAPK) {
      if(!constJR) {
        PJRR = fTR?fAPK->parDer1(getqRRel(t),t)*(*fTR)(getqRRel(t)):fAPK->parDer1(getqRRel(t),t);
        PJR[0].set(i02,iuR,PJRR);
      }
      if(!constjR)
        PjhR = fAPK->parDer2(getqRRel(t),t);
    }
    updPJ = false;
  }

  void RigidBody::updateGyroscopicAccelerations(double t) {
    VecV qdTRel = getuTRel(t);
    VecV qdRRel = fTR ? (*fTR)(qRRel)*uRRel : uRRel;
    if(fPrPK) {
      if(not(constJT and constjT)) {
        PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,t)+fPrPK->parDer1ParDer2(qTRel,t))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,t) + fPrPK->parDer2ParDer2(qTRel,t);
      }
    }
    if(fAPK) {
      if(not(constJR and constjR)) {
        if(fTR) {
          Mat3xV JRd = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t));
          MatV TRd = fTR->dirDer(qdRRel,qRRel);
          PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,t)*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
        }
        else
          PjbR = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t))*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
      }
    }
    updPjb = false;
  }

  void RigidBody::updatePositions(double t, Frame *frame) {
    frame->setPosition(R->getPosition(t) + getGlobalRelativePosition(t));
    frame->setOrientation(R->getOrientation()*APK); // APK already update to date
  }

  void RigidBody::updateVelocities(double t, Frame *frame) {
    frame->setAngularVelocity(R->getAngularVelocity(t) + getGlobalRelativeAngularVelocity(t));
    frame->setVelocity(R->getVelocity() + WvPKrel + crossProduct(R->getAngularVelocity(),getGlobalRelativePosition(t))); // WvPKrel already update to date
  }

  void RigidBody::updateAccelerations(double t, Frame *frame) {
    frame->setAcceleration(Z.getJacobianOfTranslation(t)*udall[0] + Z.getGyroscopicAccelerationOfTranslation(t));
    frame->setAngularAcceleration(Z.getJacobianOfRotation(t)*udall[0] + Z.getGyroscopicAccelerationOfRotation(t));
  }

  void RigidBody::updateJacobians0(double t, Frame *frame) {
    frame->getJacobianOfTranslation(0,false).init(0);
    frame->getJacobianOfRotation(0,false).init(0);
    frame->getJacobianOfTranslation(0,false).set(i02,Index(0,R->gethSize()-1), R->getJacobianOfTranslation(t) - tilde(getGlobalRelativePosition(t))*R->getJacobianOfRotation(t));
    frame->getJacobianOfRotation(0,false).set(i02,Index(0,R->gethSize()-1), R->getJacobianOfRotation(t));
    frame->getJacobianOfTranslation(0,false).add(i02,Index(0,gethSize(0)-1), R->getOrientation(t)*getPJT(t)*getJRel(t));
    frame->getJacobianOfRotation(0,false).add(i02,Index(0,gethSize(0)-1), frameForJacobianOfRotation->getOrientation(t)*PJR[0]*JRel[0]);
  }

  void RigidBody::updateGyroscopicAccelerations(double t, Frame *frame) {
    frame->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(t) + crossProduct(R->getGyroscopicAccelerationOfRotation(t),getGlobalRelativePosition(t)) + R->getOrientation(t)*(getPjbT(t) + getPJT(t)*getjRel(t)) + crossProduct(R->getAngularVelocity(t), 2.*getGlobalRelativeVelocity(t)+crossProduct(R->getAngularVelocity(t),getGlobalRelativePosition(t))));
    frame->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation(t) + frameForJacobianOfRotation->getOrientation(t)*(PjbR + PJR[0]*jRel) + crossProduct(R->getAngularVelocity(t), getGlobalRelativeAngularVelocity(t))); // PjbR already up to date
  }

  void RigidBody::updateJacobians2(double t, Frame *frame_) {
    for(vector<Frame*>::iterator i=frame.begin(); i!=frame.end(); i++) {
      (*i)->getJacobianOfTranslation(2,false).resize();
      (*i)->getJacobianOfRotation(2,false).resize();
    }
    if(updateByReference) {
      frame_->getJacobianOfTranslation(2,false).resize() = R->getJacobianOfTranslation(t,2) - tilde(getGlobalRelativePosition(t))*R->getJacobianOfRotation(t,2);
      frame_->getJacobianOfRotation(2,false).resize() = R->getJacobianOfRotation(t,2);
    } else {
      frame_->getJacobianOfTranslation(2,false).resize() = R->getOrientation(t)*getPJT(t);
      frame_->getJacobianOfRotation(2,false).resize() = frameForJacobianOfRotation->getOrientation(t)*PJR[0];
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
    updGC = true;
    updGJ = true;
    updWTS = true;
    updT = true;
  }

  void RigidBody::updateqRef(const Vec& ref) {
    Body::updateqRef(ref);
    if(!constraint) qRel>>q;
  }

  void RigidBody::updateuRef(const Vec& ref) {
    Body::updateuRef(ref);
    if(!constraint) uRel>>u;
  }

  void RigidBody::addFrame(FixedRelativeFrame *frame) {
    Body::addFrame(frame);
  }

  void RigidBody::addContour(RigidContour *contour) {
    Body::addContour(contour);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void RigidBody::setOpenMBVRigidBody(const shared_ptr<OpenMBV::RigidBody> &body) {
    openMBVBody=body;
  }
#endif

  void RigidBody::updateMConst(double t, int i) {
    M[i] += Mbuf;
  }

  void RigidBody::updateMNotConst(double t, int i) {
    M[i] += m*JTJ(C->getJacobianOfTranslation(t,i)) + JTMJ(getGlobalInertiaTensor(t),C->getJacobianOfRotation(t,i));
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

#ifdef HAVE_OPENMBVCPPINTERFACE
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

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(e) {
      if(!openMBVBody) setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FWeight=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if (e) {
      if(!openMBVBody) setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FArrow=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if (e) {
      if(!openMBVBody) setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      MArrow=ombv.createOpenMBV(e);
    }
#endif
  }

  DOMElement* RigidBody::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Body::writeXMLFile(parent);

    //    DOMElement * ele1 = new DOMElement( MBSIM%"frameForKinematics" );
    //    string str = string("Frame[") + getFrameForKinematics()->getName() + "]";
    //    ele1->SetAttribute("ref", str);
    //    ele0->LinkEndChild(ele1);
    //
    //    addElementText(ele0,MBSIM%"mass",getMass());
    //    if(frameForInertiaTensor)
    //      THROW_MBSIMERROR("Inertia tensor with respect to frame " + frameForInertiaTensor->getPath() + " not supported in XML. Provide inertia tensor with respect to frame C.");
    //    addElementText(ele0,MBSIM%"inertiaTensor",getInertiaTensor());
    //
    //    ele1 = new DOMElement( MBSIM%"translation" );
    //    if(getTranslation())
    //      getTranslation()->writeXMLFile(ele1);
    //    ele0->LinkEndChild(ele1);
    //
    //    ele1 = new DOMElement( MBSIM%"rotation" );
    //    if(getRotation())
    //      getRotation()->writeXMLFile(ele1);
    //    ele0->LinkEndChild(ele1);
    //
    //    ele1 = new DOMElement( MBSIM%"frames" );
    //    for(vector<Frame*>::iterator i = frame.begin()+1; i != frame.end(); ++i)
    //      (*i)->writeXMLFile(ele1);
    //    ele0->LinkEndChild( ele1 );
    //
    //    ele1 = new DOMElement( MBSIM%"contours" );
    //    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
    //      (*i)->writeXMLFile(ele1);
    //    ele0->LinkEndChild( ele1 );
    //
    //#ifdef HAVE_OPENMBVCPPINTERFACE
    //    if(getOpenMBVBody()) {
    //      ele1 = new DOMElement( MBSIM%"openMBVRigidBody" );
    //      getOpenMBVBody()->writeXMLFile(ele1);
    //
    //      if(getOpenMBVFrameOfReference()) {
    //        DOMElement * ele2 = new DOMElement( MBSIM%"frameOfReference" );
    //        string str = string("Frame[") + getOpenMBVFrameOfReference()->getName() + "]";
    //        ele2->SetAttribute("ref", str);
    //        ele1->LinkEndChild(ele2);
    //      }
    //      ele0->LinkEndChild(ele1);
    //    }
    //
    //    if(C->getOpenMBVFrame()) {
    //      ele1 = new DOMElement( MBSIM%"enableOpenMBVFrameC" );
    //      addElementText(ele1,MBSIM%"size",C->getOpenMBVFrame()->getSize());
    //      addElementText(ele1,MBSIM%"offset",C->getOpenMBVFrame()->getOffset());
    //      ele0->LinkEndChild(ele1);
    //    }
    //
    //    if(FWeight) {
    //      ele1 = new DOMElement( MBSIM%"openMBVWeightArrow" );
    //      FWeight->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //
    //    if(FArrow) {
    //      ele1 = new DOMElement( MBSIM%"openMBVJointForceArrow" );
    //      FArrow->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //
    //    if(MArrow) {
    //      ele1 = new DOMElement( MBSIM%"openMBVJointMomentArrow" );
    //      MArrow->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //#endif

    return ele0;
  }

  void RigidBody::addDependency(Constraint *constraint_) {
    Body::addDependency(constraint_);
    constraint = constraint_;
  }

  void RigidBody::setqRel(const fmatvec::Vec &q) { 
    qRel = q;
    qTRel = qRel(iqT); 
    qRRel = qRel(iqR); 
    updGC = false; 
  }

  void RigidBody::setuRel(const fmatvec::Vec &u) { 
    uRel = u; 
    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    updGC = false; 
  }

  void RigidBody::setJRel(const fmatvec::Mat &J) { 
    JRel[0] = J; 
    updGJ = false; 
  }

  void RigidBody::setjRel(const fmatvec::Vec &j) { 
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
