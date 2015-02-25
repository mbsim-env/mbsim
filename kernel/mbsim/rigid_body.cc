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
#include "rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/joint.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include "mbsim/environment.h"
#include "mbsim/constraint.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/contours/compound_contour.h"
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

  RigidBody::RigidBody(const string &name) : Body(name), m(0), coordinateTransformation(true), APK(EYE), fTR(0), fPrPK(0), fAPK(0), constraint(0), frameForJacobianOfRotation(0), frameForInertiaTensor(0), translationDependentRotation(false), constJT(false), constJR(false), constjT(false), constjR(false) {

    C=new FixedRelativeFrame("C");
    Body::addFrame(C);
    K = C;
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=C;
#endif

    updateJacobians_[0] = &RigidBody::updateJacobians0;
    updateJacobians_[1] = &RigidBody::updateJacobians1;
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
    h[j] += C->getJacobianOfTranslation(j).T()*(WF - m*C->getGyroscopicAccelerationOfTranslation(j)) + C->getJacobianOfRotation(j).T()*(WM - WThetaS*C->getGyroscopicAccelerationOfRotation(j));
  }

  void RigidBody::updatehInverseKinetics(double t, int j) {
    h[j] -= C->getJacobianOfTranslation(j).T()*(m*(C->getJacobianOfTranslation()*udall[0] + C->getGyroscopicAccelerationOfTranslation())) + C->getJacobianOfRotation(j).T()*(WThetaS*(C->getJacobianOfRotation()*udall[0] + C->getGyroscopicAccelerationOfRotation()));
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

      for(unsigned int k=0; k<contour.size(); k++) {
        if(!(contour[k]->getFrameOfReference()))
          contour[k]->setFrameOfReference(C);
        CompoundContour *c = dynamic_cast<CompoundContour*>(contour[k]);
        if(c) RBC.push_back(c);
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
        dependency.push_back(constraint);
    }
    else if(stage==relativeFrameContourLocation) {

      //RBF.push_back(C);
      for(unsigned int k=1; k<frame.size(); k++) {
        if(!((FixedRelativeFrame*) frame[k])->getFrameOfReference())
          ((FixedRelativeFrame*) frame[k])->setFrameOfReference(C);
      }
      for(unsigned int k=1; k<frame.size(); k++) {
        FixedRelativeFrame *P = (FixedRelativeFrame*)frame[k];
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

      WJTrel.resize(nu[0]);
      WJRrel.resize(nu[0]);

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;
    }
    else if(stage==unknownStage) {
      Body::init(stage);

      C->getJacobianOfTranslation(1) = PJT[1];
      C->getJacobianOfRotation(1) = PJR[1];

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
      else if(Atmp and dynamic_cast<RotationAboutAxesXYZ2<VecV>*>(Atmp->getFunction())) {
        cb = true;
        if(coordinateTransformation) {
          fTR = new RotationAboutAxesXYZMapping2<VecV>;
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
        SThetaS = JMJT(static_cast<FixedRelativeFrame*>(frameForInertiaTensor)->getRelativeOrientation(),SThetaS) - m*JTJ(tilde(static_cast<FixedRelativeFrame*>(frameForInertiaTensor)->getRelativePosition()));
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
  }

  void RigidBody::initz() {
    Object::initz();
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
    joint->connect(R,K);
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
          static_pointer_cast<OpenMBV::RigidBody>(openMBVBody)->append(data);
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
    if(fTR) TRel(iqR,iuR) = (*fTR)(qRel(iuR));
  }

  void RigidBody::updateKinematicsForSelectedFrame(double t) {

    if(fPrPK) {
      qTRel = qRel(iqT);
      uTRel = uRel(iuT);
      PrPK = (*fPrPK)(qTRel,t);
      if(!constJT) {
        PJTT = fPrPK->parDer1(qTRel,t);
        PJT[0].set(i02,iuT,PJTT);
//        cout << "JT" << endl;
      }
      if(!constjT) {
        PjhT = fPrPK->parDer2(qTRel,t);
//        cout << "jT" << endl;
      }
      WvPKrel = R->getOrientation()*(PJTT*uTRel + PjhT);
    }

    if(fAPK) {
      qRRel = qRel(iqR);
      uRRel = uRel(iuR);
      APK = (*fAPK)(qRRel,t);
      if(!constJR) {
        PJRR = fTR?fAPK->parDer1(qRRel,t)*(*fTR)(qRRel):fAPK->parDer1(qRRel,t);
        PJR[0].set(i02,iuR,PJRR);
//        cout << "JR" << endl;
      }
      if(!constjR) {
        PjhR = fAPK->parDer2(qRRel,t);
//        cout << "jr" << endl;
      }
      WomPK = frameForJacobianOfRotation->getOrientation()*(PJRR*uRRel + PjhR);
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
    if(fPrPK) {
      if(not(constJT and constjT)) {
//        cout << "jbT" << endl;
        PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,t)+fPrPK->parDer1ParDer2(qTRel,t))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,t) + fPrPK->parDer2ParDer2(qTRel,t);
      }
    }
    if(fAPK) {
      if(not(constJR and constjR)) {
//        cout << "jbR" << endl;
        if(fTR) {
          Mat3xV JRd = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t));
          MatV TRd = fTR->dirDer(qdRRel,qRRel);
          PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,t)*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
        }
        else
          PjbR = (fAPK->parDer1DirDer1(qdRRel,qRRel,t)+fAPK->parDer1ParDer2(qRRel,t))*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,t) + fAPK->parDer2ParDer2(qRRel,t);
      }
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

    //compute inertia in world frame
    WThetaS = JTMJ(SThetaS,C->getOrientation().T());

    WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
    WM = crossProduct(WThetaS*C->getAngularVelocity(),C->getAngularVelocity()) ;
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

#ifdef HAVE_OPENMBVCPPINTERFACE
  void RigidBody::setOpenMBVRigidBody(const shared_ptr<OpenMBV::RigidBody> &body) {
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

    if(K!=C) {
      C->updateOrientation();
      C->updatePosition();
    }

    if(P!=C) {
      ((FixedRelativeFrame*)P)->updateOrientation();
      ((FixedRelativeFrame*)P)->updatePosition();
    }
  }

  void RigidBody::updateRelativeJacobians(double t, Frame *P) {

    if(fPrPK) {
      if(!constJT) {
        PJTT = fPrPK->parDer1(qTRel,t);
        PJT[0].set(i02,iuT,PJTT);
      }
      if(!constjT) {
        PjhT = fPrPK->parDer2(qTRel,t);
      }
    }

    if(fAPK) {
      //if(fAPK->hasVariableJacobian())
      if(!constJR) {
        PJRR = fTR?fAPK->parDer1(qRRel,t)*(*fTR)(qRRel):fAPK->parDer1(qRRel,t);
        PJR[0].set(i02,iuR,PJRR);
      }
      if(!constjR) {
        PjhR = fAPK->parDer2(qRRel,t);
      }
    }

    WJRrel = frameForJacobianOfRotation->getOrientation()*PJR[0];
    WJTrel = R->getOrientation()*PJT[0];

    K->setVelocity(R->getOrientation()*PjhT+R->getVelocity() + crossProduct(R->getAngularVelocity(),WrPK));
    K->setAngularVelocity(frameForJacobianOfRotation->getOrientation()*PjhR + R->getAngularVelocity());

    if(K!=C) {
      C->updateAngularVelocity();
      C->updateVelocity();
      WJTrel -= tilde(C->getWrRP())*WJRrel;
    }

    if(P!=C) {
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
      if(not(constJT and constjT)) {
        PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,t)+fPrPK->parDer1ParDer2(qTRel,t))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,t) + fPrPK->parDer2ParDer2(qTRel,t);
      }
//      Mat3xV JT = fPrPK->parDer1(qTRel,t);
      WvPKrel = R->getOrientation()*(PJTT*uTRel + PjhT);
    }
    if(fAPK) {
      uRRel = uRel(iuR);
      VecV qdRRel = uRRel;
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
//      Mat3xV JR = fAPK->parDer1(qRRel,t);
      WomPK = frameForJacobianOfRotation->getOrientation()*(PJRR*uRRel + PjhR);
    }

    // TODO prüfen ob Optimierungspotential

    SqrMat3 tWrPK = tilde(WrPK);

    K->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() - tWrPK*R->getGyroscopicAccelerationOfRotation() + R->getOrientation()*PjbT + crossProduct(R->getAngularVelocity(), 2.*WvPKrel+crossProduct(R->getAngularVelocity(),WrPK)));
    K->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->getOrientation()*PjbR + crossProduct(R->getAngularVelocity(), WomPK));

    K->getJacobianOfTranslation().set(i02,Index(0,R->getJacobianOfTranslation().cols()-1), R->getJacobianOfTranslation() - tWrPK*R->getJacobianOfRotation());
    K->getJacobianOfRotation().set(i02,Index(0,R->getJacobianOfRotation().cols()-1), R->getJacobianOfRotation());

   if(K!=C)
     C->updateJacobians();

   if(P!=C) 
     ((FixedRelativeFrame*)P)->updateJacobians();
  }

  void RigidBody::updateRelativeJacobians(double t, Frame *P, Mat3xV &WJTrel0, Mat3xV &WJRrel0) {

    WJTrel0 -= tilde(WrPK)*WJRrel0;

    if(K!=C) {
      WJTrel0 -= tilde(C->getWrRP())*WJRrel0;
    }

    // TODO: Zusammenfassen
    if(P!=C)
      WJTrel0 -= tilde(((FixedRelativeFrame*)P)->getWrRP())*WJRrel0;
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
      Contour *c=ObjectFactory::createAndInit<Contour>(e);
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

}
