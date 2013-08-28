/* Copyright (C) 2004-2010 MBSim Development Team
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
#include "rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include "mbsim/environment.h"
#include "mbsim/constraint.h"
#include "mbsim/utils/utils.h"
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
//#include "mbsim/utils/symbolic_function.h"
#endif
#include "mbsim/contours/compound_contour.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  Range<Var,Var> i02(0,2);

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, RigidBody, MBSIMNS"RigidBody")

  RigidBody::RigidBody(const string &name) : Body(name), m(0), cb(false), APK(EYE), fTR(0), fPrPK(0), fAPK(0), constraint(0), frameForJacobianOfRotation(0), frameForInertiaTensor(0), translationDependentRotation(false), constantJacobianOfRotation(false) {

    C=new FixedRelativeFrame("C");
    Body::addFrame(C);
    K = C;
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=C;
    FWeight = 0;
    FArrow = 0;
    MArrow = 0;
#endif

    updateJacobians_[0] = &RigidBody::updateJacobians0;
    updateJacobians_[1] = &RigidBody::updateJacobians1;
  }

  RigidBody::~RigidBody() {
    if(fPrPK) { delete fPrPK; fPrPK=0; }
    if(fAPK) { delete fAPK; fAPK=0; }
  }

  void RigidBody::setFrameForKinematics(Frame *frame) { 
    K = dynamic_cast<FixedRelativeFrame*>(frame); 
    assert(K);
  }

  void RigidBody::updateh(double t, int j) {

    Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity() - m*C->getGyroscopicAccelerationOfTranslation(j);
    Vec3 WM = crossProduct(WThetaS*C->getAngularVelocity(),C->getAngularVelocity()) - WThetaS*C->getGyroscopicAccelerationOfRotation(j);

    h[j] += C->getJacobianOfTranslation(j).T()*WF + C->getJacobianOfRotation(j).T()*WM;
  }

  void RigidBody::updateh0Fromh1(double t) {
    h[0] += C->getJacobianOfTranslation(0).T()*(h[1](0,2) - m*C->getGyroscopicAccelerationOfTranslation()) + C->getJacobianOfRotation(0).T()*(h[1](3,5) - WThetaS*C->getGyroscopicAccelerationOfRotation());
  }

  void RigidBody::updateW0FromW1(double t) {
    W[0] += C->getJacobianOfTranslation(0).T()*W[1](i02,Index(0,W[1].cols()-1)) + C->getJacobianOfRotation(0).T()*W[1](Index(3,5),Index(0,W[1].cols()-1));
  }

  void RigidBody::updateV0FromV1(double t) {
    V[0] += C->getJacobianOfTranslation(0).T()*V[1](i02,Index(0,V[1].cols()-1)) + C->getJacobianOfRotation(0).T()*V[1](Index(3,5),Index(0,V[1].cols()-1));
  }

  void RigidBody::updatehInverseKinetics(double t, int j) {
    VecV buf = C->getJacobianOfTranslation(j).T()*(m*(C->getJacobianOfTranslation()*udall[0] + C->getGyroscopicAccelerationOfTranslation())) + C->getJacobianOfRotation(j).T()*(WThetaS*(C->getJacobianOfRotation()*udall[0] + C->getGyroscopicAccelerationOfRotation()));
    h[j] -= buf;
  }

  void RigidBody::updateStateDerivativeDependentVariables(double t) {
    for(unsigned int i=0; i<frame.size(); i++)
      ((FixedRelativeFrame*)frame[i])->updateStateDerivativeDependentVariables(udall[0]);
    for(unsigned int i=0; i<RBC.size(); i++)
      RBC[i]->updateStateDerivativeDependentVariables(udall[0],t);
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
      Body::init(stage);

      int nqT=0, nqR=0, nuT=0, nuR=0;
      nq=0, nu[0]=0;
      if(fPrPK) {
        nqT = fPrPK->getArg1Size();
        nuT = fPrPK->getArg1Size(); // TODO fTT->getArg1Size()
      }
      if(fAPK) {
        nqR = fAPK->getArg1Size();
        nuR = fAPK->getArg1Size(); // TODO fTR->getArg1Size()
      }

      if(translationDependentRotation) {
        assert(nqT = nqR);
        assert(nuT = nuR);
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
        dependency.push_back(constraint);
    }
    else if(stage==relativeFrameContourLocation) {

      //RBF.push_back(C);
      for(unsigned int k=1; k<frame.size(); k++) {
        FixedRelativeFrame *P = (FixedRelativeFrame*)frame[k];
        if(!(P->getFrameOfReference()))
          P->setFrameOfReference(C);
        const FixedRelativeFrame *R = P;
        do {
          R = static_cast<const FixedRelativeFrame*>(R->getFrameOfReference());
          P->setRelativePosition(R->getRelativePosition() + R->getRelativeOrientation()*P->getRelativePosition());
          P->setRelativeOrientation(R->getRelativeOrientation()*P->getRelativeOrientation());
        } while(R!=C);
        P->setFrameOfReference(C);
        if(P!=K)
          RBF.push_back(P);
      }
      if(K!=C) {
        C->setFrameOfReference(K);
        C->setRelativeOrientation(K->getRelativeOrientation().T());
        C->setRelativePosition(-(C->getRelativeOrientation()*K->getRelativePosition()));
      }

      for(unsigned int k=0; k<contour.size(); k++) {
        if(!(contour[k]->getFrameOfReference()))
          contour[k]->setFrameOfReference(C);
        CompoundContour *c = dynamic_cast<CompoundContour*>(contour[k]);
        if(c) RBC.push_back(c);
      }
    }
    else if(stage==resize) {
      Body::init(stage);

      if(fPrPK) {
//        fPrPK->init();
      }
      if(fAPK) {
//        fAPK->setKOSY(cb);
//        fAPK->init();
      }

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

      WJTrel.resize(nu[0]);
      WJRrel.resize(nu[0]);

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;
    }
    else if(stage==MBSim::unknownStage) {
      Body::init(stage);

      C->getJacobianOfTranslation(1) = PJT[1];
      C->getJacobianOfRotation(1) = PJR[1];

      if(dynamic_cast<TCardanAngles<VecV>*>(fTR))
        constantJacobianOfRotation = true;
      else if(dynamic_cast<TCardanAngles2<VecV>*>(fTR)) {
        constantJacobianOfRotation = true;
        cb = true;
      }

      if(cb) {
        frameForJacobianOfRotation = K;
        // TODO
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

      // TODO
//          if(fPrPK) {
//            fPrPK->updateJacobian(qRel(iqT),0);
//            PJT[0].set(i02,iuT,fPrPK->getJacobian());
//          }
          // TODO
//          if(fAPK) {
//            fAPK->updateJacobian(qRel(iqR),0);
//            PJR[0].set(i02,iuR,fAPK->getJacobian());
//          }
      if(frameForInertiaTensor && frameForInertiaTensor!=C)
        SThetaS = JMJT(static_cast<FixedRelativeFrame*>(frameForInertiaTensor)->getRelativeOrientation(),SThetaS) - m*JTJ(tilde(static_cast<FixedRelativeFrame*>(frameForInertiaTensor)->getRelativePosition()));
    }
    else if(stage==MBSim::plot) {
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
  }

  void RigidBody::initz() {
    Object::initz();
    if(!constraint) qRel>>q;
    if(!constraint) uRel>>u;
  }

  void RigidBody::setUpInverseKinetics() {
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+R->getParent()->getName()+"_"+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    joint->setForceDirection(Mat3xV(3,EYE));
    joint->setMomentDirection(Mat3xV(3,EYE));
    joint->connect(R,K);
    joint->setBody(this);
    if(FArrow)
      joint->setOpenMBVForceArrow(FArrow);
    if(MArrow)
      joint->setOpenMBVMomentArrow(MArrow);
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
          Vec3 WrOS=C->getPosition();
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
          Vec3 WrOS=openMBVFrame->getPosition();
          Vec3 cardan=AIK2Cardan(openMBVFrame->getOrientation());
          data.push_back(WrOS(0));
          data.push_back(WrOS(1));
          data.push_back(WrOS(2));
          data.push_back(cardan(0));
          data.push_back(cardan(1));
          data.push_back(cardan(2));
          data.push_back(0);
          ((OpenMBV::RigidBody*)openMBVBody)->append(data);
        }
      }
#endif
      Body::plot(t,dt);
    }
  }

  void RigidBody::updateqd(double t) {
    if(!constraint) {
      qd(iqT) = uRel(iuT);
      if(fTR)
        qd(iqR) = (*fTR)(qRel(iuR))*uRel(iuR);
      else
        qd(iqR) = uRel(iuR);
    }
  }

  void RigidBody::updatedq(double t, double dt) {
    if(!constraint) {
      qd(iqT) = uRel(iuT)*dt;
      if(fTR)
        qd(iqR) = (*fTR)(qRel(iuR))*uRel(iuR)*dt;
      else
        qd(iqR) = uRel(iuR)*dt;
    }
  }

  void RigidBody::updateT(double t) {
    if(!constraint) {
      // TODO
      //if(fPrPK) T(iqT,iuT) = fPrPK->getT();
      if(fTR) T(iqR,iuR) = (*fTR)(qRel(iuR));
    }
  }

  void RigidBody::updateKinematicsForSelectedFrame(double t) {

    if(fPrPK) {
      qTRel = qRel(iqT);
      uTRel = uRel(iuT);
      PrPK = (*fPrPK)(qTRel,t);
      //if(fPrPK->hasVariableJacobian())
      Mat3xV JT = fPrPK->parDer1(qTRel,t);
      Vec3 jT = fPrPK->parDer2(qTRel,t);
      PJT[0].set(i02,iuT,JT);
      WvPKrel = R->getOrientation()*(JT*uTRel + jT);
    }

    if(fAPK) {
      qRRel = qRel(iqR);
      uRRel = uRel(iuR);
      APK = (*fAPK)(qRRel,t);
      //if(fAPK->hasVariableJacobian())
      Mat3xV JR;
      if(fTR) 
        JR = fAPK->parDer1(qRRel,t)*(*fTR)(qRRel);
      else
        JR = fAPK->parDer1(qRRel,t);
      Vec3 jR = fAPK->parDer2(qRRel,t);
      PJR[0].set(i02,iuR,JR);
      WomPK = frameForJacobianOfRotation->getOrientation()*(JR*uRRel + jR);
    }

    K->setOrientation(R->getOrientation()*APK);
    WrPK = R->getOrientation()*PrPK;

    K->setAngularVelocity(R->getAngularVelocity() + WomPK);
    K->setPosition(WrPK + R->getPosition());
    K->setVelocity(R->getVelocity() + WvPKrel + crossProduct(R->getAngularVelocity(),WrPK));
  }

  void RigidBody::updateJacobiansForSelectedFrame0(double t) {
    K->getJacobianOfTranslation().init(0);
    K->getJacobianOfRotation().init(0);

    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    VecV qdTRel = uTRel;
    VecV qdRRel = fTR ? (*fTR)(qRRel)*uRRel : uRRel;
    if(fPrPK) PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,t)+fPrPK->parDer1ParDer2(qTRel,t))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,t) + fPrPK->parDer2ParDer2(qTRel,t);
    if(fAPK) {
      if(constantJacobianOfRotation)
        PjbR.init(0);
      else if(fTR) {
        Mat3xV JRd = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t));
        MatV TRd = fTR->dirDer(qdRRel,qRRel);
        PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,t)*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
      }
      else
        PjbR = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t))*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
    }

    SqrMat3 tWrPK = tilde(WrPK);
    K->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() - tWrPK*R->getGyroscopicAccelerationOfRotation() + R->getOrientation()*(PjbT + PJT[0]*jRel) + crossProduct(R->getAngularVelocity(), 2.*WvPKrel+crossProduct(R->getAngularVelocity(),WrPK)));
    K->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->getOrientation()*(PjbR + PJR[0]*jRel) + crossProduct(R->getAngularVelocity(), WomPK));

    K->getJacobianOfTranslation().set(i02,Index(0,R->getJacobianOfTranslation().cols()-1), R->getJacobianOfTranslation() - tWrPK*R->getJacobianOfRotation());
    K->getJacobianOfRotation().set(i02,Index(0,R->getJacobianOfRotation().cols()-1), R->getJacobianOfRotation());

    K->getJacobianOfTranslation().add(i02,Index(0,gethSize(0)-1), R->getOrientation()*PJT[0]*JRel[0]);
    K->getJacobianOfRotation().add(i02,Index(0,gethSize(0)-1), frameForJacobianOfRotation->getOrientation()*PJR[0]*JRel[0]);
  }

  void RigidBody::updateKinematicsForRemainingFramesAndContours(double t) {
    if(K != C) C->updateStateDependentVariables();
    for(unsigned int i=0; i<RBF.size(); i++)
      RBF[i]->updateStateDependentVariables();
    for(unsigned int i=0; i<RBC.size(); i++)
      RBC[i]->updateStateDependentVariables(t);

    WThetaS = JTMJ(SThetaS,C->getOrientation().T());
  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours(double t, int j) {
    if(K != C) C->updateJacobians();
    for(unsigned int i=0; i<RBF.size(); i++)
      RBF[i]->updateJacobians(j);
    for(unsigned int i=0; i<RBC.size(); i++)
      RBC[i]->updateJacobians(t,j);
  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours1(double t) {
    if(K != C) {
      K->updateRelativePosition();
      K->updateJacobians(1);
    }
    for(unsigned int i=0; i<RBF.size(); i++) {
      RBF[i]->updateRelativePosition();
      RBF[i]->updateJacobians(1);
    }
    for(unsigned int i=0; i<RBC.size(); i++)
      RBC[i]->updateJacobians(t,1);
  }

  void RigidBody::updateqRef(const Vec& ref) {
    Object::updateqRef(ref);
    if(!constraint) qRel>>q;
  }

  void RigidBody::updateuRef(const Vec& ref) {
    Object::updateuRef(ref);
    if(!constraint) uRel>>u;
  }

  void RigidBody::addFrame(FixedRelativeFrame *frame_) {
    Body::addFrame(frame_);
  }

  void RigidBody::addFrame(Frame *frame_, const Vec3 &RrRF, const SqrMat3 &ARF, const Frame* refFrame) {
    Deprecated::registerMessage("Using RigidBody::addFrame(Frame*, const Vec3&, const SqrMat3&, const Frame*) is deprecated, create a FixedRelativeFrame instead and add is using addFrame(FixedRelativeFrame*).");
    FixedRelativeFrame *rigidBodyFrame = new FixedRelativeFrame(frame_->getName(),RrRF,ARF,refFrame);
    if(frame_->getOpenMBVFrame())
      rigidBodyFrame->enableOpenMBV(frame_->getOpenMBVFrame()->getSize(), frame_->getOpenMBVFrame()->getOffset());
    addFrame(rigidBodyFrame);
  }

  void RigidBody::addFrame(const string &str, const Vec3 &RrRF, const SqrMat3 &ARF, const Frame* refFrame) {
    Deprecated::registerMessage("Using RigidBody::addFrame(const string&, const Vec3&, const SqrMat3&, const Frame*) is deprecated, create a FixedRelativeFrame instead and add is using addFrame(FixedRelativeFrame*).");
    FixedRelativeFrame *rigidBodyFrame = new FixedRelativeFrame(str,RrRF,ARF,refFrame);
    addFrame(rigidBodyFrame);
  }

  void RigidBody::addContour(Contour* contour_, const Vec3 &RrRC, const SqrMat3 &ARC, const Frame* refFrame) {
    Deprecated::registerMessage("Using RigidBody::addCongour(Contour*, const Vec3&, const SqrMat3&, const Frame*) is deprecated, create a Contour instead and add is using addContour(Contour*).");
    stringstream frameName;
    frameName << "ContourFrame" << contour.size();
    Frame *contourFrame;
    if(!refFrame && fabs(RrRC(0))<1e-10 && fabs(RrRC(1))<1e-10 && fabs(RrRC(2))<1e-10 && 
      fabs(ARC(0,0)-1)<1e-10 && fabs(ARC(1,1)-1)<1e-10 && fabs(ARC(2,2)-1)<1e-10)
      contourFrame = C;
    else {
      contourFrame = new FixedRelativeFrame(frameName.str(),RrRC,ARC,refFrame);
      addFrame((FixedRelativeFrame*)contourFrame);
    }
    contour_->setFrameOfReference(contourFrame);
    Body::addContour(contour_);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void RigidBody::setOpenMBVRigidBody(OpenMBV::RigidBody* body) {
    openMBVBody=body;
  }
#endif

  void RigidBody::updateMConst(double t, int i) {
    M[i] += Mbuf;
  }

  void RigidBody::updateMNotConst(double t, int i) {
    M[i] += m*JTJ(C->getJacobianOfTranslation(i)) + JTMJ(WThetaS,C->getJacobianOfRotation(i));
  }

  void RigidBody::updatePositionAndOrientationOfFrame(double t, Frame *P) {

    if(fPrPK) {
      qTRel = qRel(iqT);
      uTRel = uRel(iuT);
      PrPK = (*fPrPK)(qTRel,t);
    }
    if(fAPK) {
      qRRel = qRel(iqR);
      uRRel = uRel(iuR);
      APK = (*fAPK)(qRRel,t);
    }

    K->setOrientation(R->getOrientation()*APK);

    WrPK = R->getOrientation()*PrPK;

    K->setPosition(WrPK + R->getPosition());

    if(K != C) {
      C->updateOrientation();
      C->updatePosition();
      K->updateRelativePosition();
    }

    if(P!=C && P!=K) {
      ((FixedRelativeFrame*)P)->updateOrientation();
      ((FixedRelativeFrame*)P)->updatePosition();
    }
  }

  void RigidBody::updateRelativeJacobians(double t, Frame *P) {

    if(fPrPK) {
      //if(fPrPK->hasVariableJacobian())
      Mat3xV JT = fPrPK->parDer1(qTRel,t);
      PJT[0].set(i02,iuT,JT);
      PjhT = fPrPK->parDer2(qTRel,t);
    }

    if(fAPK) {
      //if(fAPK->hasVariableJacobian())
      Mat3xV JR;
      if(fTR) 
        JR = fAPK->parDer1(qRRel,t)*(*fTR)(qRRel);
      else
        JR = fAPK->parDer1(qRRel,t);
      PJR[0].set(i02,iuR,JR);
      PjhR = fAPK->parDer2(qRRel,t);
    }

    WJRrel = frameForJacobianOfRotation->getOrientation()*PJR[0];
    WJTrel = R->getOrientation()*PJT[0];

    K->setVelocity(R->getOrientation()*PjhT+R->getVelocity() + crossProduct(R->getAngularVelocity(),WrPK));
    K->setAngularVelocity(frameForJacobianOfRotation->getOrientation()*PjhR + R->getAngularVelocity());

    if(K != C) {
      C->updateAngularVelocity();
      C->updateVelocity();
      WJTrel += tilde(K->getWrRP())*WJRrel;
    }

    if(P!=C && P!=K) {
      ((FixedRelativeFrame*)P)->updateAngularVelocity();
      ((FixedRelativeFrame*)P)->updateVelocity();
      WJTrel -= tilde(((FixedRelativeFrame*)P)->getWrRP())*WJRrel;
    }
  }

  void RigidBody::updateAccelerations(double t, Frame *P) {
    K->getJacobianOfTranslation().init(0);
    K->getJacobianOfRotation().init(0);

    if(fPrPK) {
      uTRel = uRel(iuT);
      VecV qdTRel = uTRel;
      PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,t)+fPrPK->parDer1ParDer2(qTRel,t))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,t) + fPrPK->parDer2ParDer2(qTRel,t);
      Mat3xV JT = fPrPK->parDer1(qTRel,t);
      WvPKrel = R->getOrientation()*(JT*uTRel + PjhT);
    }
    if(fAPK) {
      uRRel = uRel(iuR);
      VecV qdRRel = uRRel;
      if(fAPK) {
        if(constantJacobianOfRotation)
          PjbR.init(0);
        else if(fTR) {
          Mat3xV JRd = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t));
          MatV TRd = fTR->dirDer(qdRRel,qRRel);
          PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,t)*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
        }
        else
      PjbR = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t))*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
    }
      Mat3xV JR = fAPK->parDer1(qRRel,t);
      WomPK = frameForJacobianOfRotation->getOrientation()*(JR*uRRel + PjhR);
    }

    // TODO prüfen ob Optimierungspotential

    SqrMat3 tWrPK = tilde(WrPK);

    K->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() - tWrPK*R->getGyroscopicAccelerationOfRotation() + R->getOrientation()*PjbT + crossProduct(R->getAngularVelocity(), 2.*WvPKrel+crossProduct(R->getAngularVelocity(),WrPK)));
    K->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->getOrientation()*PjbR + crossProduct(R->getAngularVelocity(), WomPK));

    K->getJacobianOfTranslation().set(i02,Index(0,R->getJacobianOfTranslation().cols()-1), R->getJacobianOfTranslation() - tWrPK*R->getJacobianOfRotation());
    K->getJacobianOfRotation().set(i02,Index(0,R->getJacobianOfRotation().cols()-1), R->getJacobianOfRotation());

   if(K != C) C->updateJacobians();
   if(P!=C && P!=K)
     ((FixedRelativeFrame*)P)->updateJacobians();
  }

  void RigidBody::updateRelativeJacobians(double t, Frame *P, Mat3xV &WJTrel0, Mat3xV &WJRrel0) {

    if(K != C) {
      WJTrel0 += tilde(K->getWrRP())*WJRrel0;
    }

    // TODO: Zusammenfassen
    if(P!=C && P!=K)
      WJTrel0 -= tilde(((FixedRelativeFrame*)P)->getWrRP())*WJRrel0;
  }

  void RigidBody::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMNS"frame") {
      Deprecated::registerMessage("Using the <mbsim:frame> element is deprecated, use the <mbsim:Frame> element instead.", e);
      TiXmlElement *ec=e->FirstChildElement();
      FixedRelativeFrame *f=new FixedRelativeFrame(ec->Attribute("name"));
      addFrame(f);
      f->initializeUsingXML(ec);
      ec=ec->NextSiblingElement();
      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
        f->setFrameOfReference(string("../")+ec->Attribute("ref"));
        ec=ec->NextSiblingElement();
      }
      f->setRelativePosition(getVec3(ec));
      ec=ec->NextSiblingElement();
      f->setRelativeOrientation(getSqrMat3(ec));
      e=e->NextSiblingElement();
    }
    while(e && e->ValueStr()==MBSIMNS"FixedRelativeFrame") {
      FixedRelativeFrame *f=new FixedRelativeFrame(e->Attribute("name"));
      addFrame(f);
      f->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }

    // contours
    e=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMNS"contour") {
      Deprecated::registerMessage("Using the <mbsim:contour> element is deprecated, use the <mbsim:Contour> element instead.", e);
      TiXmlElement *ec=e->FirstChildElement();

      Contour *c=ObjectFactory<Element>::create<Contour>(ec);
      c->initializeUsingXML(ec);
      ec=ec->NextSiblingElement();
      string refF;
      if(ec) {
        if(ec->ValueStr()==MBSIMNS"frameOfReference") {
          refF = string("../")+ec->Attribute("ref");
          ec=ec->NextSiblingElement();
        }
        Vec3 RrRC = getVec3(ec);
        ec=ec->NextSiblingElement();
        SqrMat3 ARC = getSqrMat3(ec);
        e=e->NextSiblingElement();
        stringstream frameName;
        frameName << "ContourFrame" << contour.size();
        Frame *contourFrame;
        if(refF=="" && fabs(RrRC(0))<1e-10 && fabs(RrRC(1))<1e-10 && fabs(RrRC(2))<1e-10 && 
            fabs(ARC(0,0)-1)<1e-10 && fabs(ARC(1,1)-1)<1e-10 && fabs(ARC(2,2)-1)<1e-10)
          contourFrame = C;
        else {
          contourFrame = new FixedRelativeFrame(frameName.str());
          ((FixedRelativeFrame*)contourFrame)->setFrameOfReference(refF);
          ((FixedRelativeFrame*)contourFrame)->setRelativePosition(RrRC);
          ((FixedRelativeFrame*)contourFrame)->setRelativeOrientation(ARC);
          addFrame((FixedRelativeFrame*)contourFrame);
        }
        c->setFrameOfReference(contourFrame);
      }
      addContour(c);
    }
    while(e) {
      Contour *c=ObjectFactory<Element>::create<Contour>(e);
      addContour(c);
      c->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }

    e=element->FirstChildElement(MBSIMNS"frameForKinematics");
    if(e) setFrameForKinematics(getByPath<Frame>(e->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=element->FirstChildElement(MBSIMNS"mass");
    setMass(getDouble(e));
    e=element->FirstChildElement(MBSIMNS"inertiaTensor");
    setInertiaTensor(getSymMat3(e));
    e=element->FirstChildElement(MBSIMNS"translation");
    Function<Vec3(VecV,double)> *trans=ObjectFactory<FunctionBase>::create<Function<Vec3(VecV,double)> >(e->FirstChildElement(),false);
    if(trans) {
      trans->initializeUsingXML(e->FirstChildElement());
      setTranslation(trans);
    } else {
      Function<Vec3(VecV)> *trans=ObjectFactory<FunctionBase>::create<Function<Vec3(VecV)> >(e->FirstChildElement(),false);
      if(trans) {
        trans->initializeUsingXML(e->FirstChildElement());
        setTranslation(trans);
      }
      else {
        Function<Vec3(double)> *trans=ObjectFactory<FunctionBase>::create<Function<Vec3(double)> >(e->FirstChildElement(),false);
        if(trans) {
          trans->initializeUsingXML(e->FirstChildElement());
          setTranslation(trans);
        }
      }
    }
    e=element->FirstChildElement(MBSIMNS"rotation");
    Function<RotMat3(VecV,double)> *rot=ObjectFactory<FunctionBase>::create<Function<RotMat3(VecV,double)> >(e->FirstChildElement(),false);
    if(rot) {
      rot->initializeUsingXML(e->FirstChildElement());
      TiXmlElement *ee=e->FirstChildElement(MBSIMNS"isDependent");
      bool dep = false;
      if(ee) dep = getBool(ee);
      setRotation(rot,dep);
    } else {
      Function<RotMat3(VecV)> *rot=ObjectFactory<FunctionBase>::create<Function<RotMat3(VecV)> >(e->FirstChildElement(),false);
      if(rot) {
        rot->initializeUsingXML(e->FirstChildElement());
        TiXmlElement *ee=e->FirstChildElement(MBSIMNS"isDependent");
        bool dep = false;
        if(ee) dep = getBool(ee);
        setRotation(rot,dep);
      }
      else {
        Function<RotMat3(double)> *rot=ObjectFactory<FunctionBase>::create<Function<RotMat3(double)> >(e->FirstChildElement(),false);
        if(rot) {
          rot->initializeUsingXML(e->FirstChildElement());
          TiXmlElement *ee=e->FirstChildElement(MBSIMNS"isDependent");
          bool dep = false;
          if(ee) dep = getBool(ee);
          setRotation(rot,dep);
        }
      }
    }
    e=element->FirstChildElement(MBSIMNS"rotationMapping");
    if(e) {
      Function<MatV(VecV)> *TR=ObjectFactory<FunctionBase>::create<Function<MatV(VecV)> >(e->FirstChildElement());
      if(TR) {
        TR->initializeUsingXML(e->FirstChildElement());
        setRotationMapping(TR);
      }
    }

    e=element->FirstChildElement(MBSIMNS"isFrameOfBodyForRotation");
    if(e) isFrameOfBodyForRotation(getBool(e));

    // END
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
    if(e) {
      OpenMBV::RigidBody *rb=OpenMBV::ObjectFactory::create<OpenMBV::RigidBody>(e->FirstChildElement());
      setOpenMBVRigidBody(rb);
      rb->initializeUsingXML(e->FirstChildElement());

      if (e->FirstChildElement(MBSIMNS"frameOfReference"))
        setOpenMBVFrameOfReference(getByPath<Frame>(e->FirstChildElement(MBSIMNS"frameOfReference")->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
    }

    e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameC");
    if(e) {
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      C->enableOpenMBV(getDouble(e->FirstChildElement(MBSIMNS"size")),
          getDouble(e->FirstChildElement(MBSIMNS"offset")));

      // pass a OPENMBV_ID processing instruction to the OpenMBV Frame object
      for(TiXmlNode *child=e->FirstChild(); child; child=child->NextSibling()) {
        TiXmlUnknown *unknown=child->ToUnknown();
        const size_t length=strlen("?OPENMBV_ID ");
        if(unknown && unknown->ValueStr().substr(0, length)=="?OPENMBV_ID ")
          C->getOpenMBVFrame()->setID(unknown->ValueStr().substr(length, unknown->ValueStr().length()-length-1));
      }
    }

    e=element->FirstChildElement(MBSIMNS"openMBVWeightArrow");
    if(e) {
      OpenMBV::Arrow *arrow=OpenMBV::ObjectFactory::create<OpenMBV::Arrow>(e->FirstChildElement());
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVWeightArrow(arrow);
    }

    e=element->FirstChildElement(MBSIMNS"openMBVJointForceArrow");
    if(e) {
      OpenMBV::Arrow *arrow=OpenMBV::ObjectFactory::create<OpenMBV::Arrow>(e->FirstChildElement());
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVJointForceArrow(arrow);
    }

    e=element->FirstChildElement(MBSIMNS"openMBVJointMomentArrow");
    if(e) {
      OpenMBV::Arrow *arrow=OpenMBV::ObjectFactory::create<OpenMBV::Arrow>(e->FirstChildElement());
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVJointMomentArrow(arrow);
    }
#endif
  }

  TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Body::writeXMLFile(parent);

    TiXmlElement * ele1 = new TiXmlElement( MBSIMNS"frameForKinematics" );
    string str = string("Frame[") + getFrameForKinematics()->getName() + "]";
    ele1->SetAttribute("ref", str);
    ele0->LinkEndChild(ele1);

    addElementText(ele0,MBSIMNS"mass",getMass());
    if(frameForInertiaTensor)
      throw MBSimError("Inertia tensor with respect to frame " + frameForInertiaTensor->getName() + " not supported in XML. Provide inertia tensor with respect to frame C.");
    addElementText(ele0,MBSIMNS"inertiaTensor",getInertiaTensor());

    ele1 = new TiXmlElement( MBSIMNS"translation" );
    if(getTranslation()) 
      getTranslation()->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);

    ele1 = new TiXmlElement( MBSIMNS"rotation" );
    if(getRotation()) 
      getRotation()->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);

    ele1 = new TiXmlElement( MBSIMNS"frames" );
    for(vector<Frame*>::iterator i = frame.begin()+1; i != frame.end(); ++i) 
      (*i)->writeXMLFile(ele1);
    ele0->LinkEndChild( ele1 );

    ele1 = new TiXmlElement( MBSIMNS"contours" );
    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
      (*i)->writeXMLFile(ele1);
    ele0->LinkEndChild( ele1 );

#ifdef HAVE_OPENMBVCPPINTERFACE
    if(getOpenMBVBody()) {
      ele1 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
      getOpenMBVBody()->writeXMLFile(ele1);

      if(getOpenMBVFrameOfReference()) {
        TiXmlElement * ele2 = new TiXmlElement( MBSIMNS"frameOfReference" );
        string str = string("Frame[") + getOpenMBVFrameOfReference()->getName() + "]";
        ele2->SetAttribute("ref", str);
        ele1->LinkEndChild(ele2);
      }
      ele0->LinkEndChild(ele1);
    }

    if(C->getOpenMBVFrame()) {
      ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
      addElementText(ele1,MBSIMNS"size",C->getOpenMBVFrame()->getSize());
      addElementText(ele1,MBSIMNS"offset",C->getOpenMBVFrame()->getOffset());
      ele0->LinkEndChild(ele1);
    }

    if(FWeight) {
      ele1 = new TiXmlElement( MBSIMNS"openMBVWeightArrow" );
      FWeight->writeXMLFile(ele1);
      ele0->LinkEndChild(ele1);
    }

    if(FArrow) {
      ele1 = new TiXmlElement( MBSIMNS"openMBVJointForceArrow" );
      FArrow->writeXMLFile(ele1);
      ele0->LinkEndChild(ele1);
    }

    if(MArrow) {
      ele1 = new TiXmlElement( MBSIMNS"openMBVJointMomentArrow" );
      MArrow->writeXMLFile(ele1);
      ele0->LinkEndChild(ele1);
    }
#endif

    return ele0;
  }

}
