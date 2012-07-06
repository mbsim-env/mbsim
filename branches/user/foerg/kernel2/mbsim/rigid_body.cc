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

#define FMATVEC_NO_BOUNDS_CHECK
#define FMATVEC_NO_SIZE_CHECK

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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), m(0), iKinematics(-1), iInertia(-1), cb(false), APK(EYE), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0), constraint(0) {

    C=new Frame("C");
    Body::addFrame(C);
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=C;
#endif

    SrSF.push_back(Vec3());
    WrSF.push_back(Vec3());
    ASF.push_back(SqrMat3(EYE));

    updateJacobians_[0] = &RigidBody::updateJacobians0;
    updateJacobians_[1] = &RigidBody::updateJacobians1;
  }

  RigidBody::~RigidBody() {
    if(fT) { delete fT; fT=0; }
    if(fPrPK) { delete fPrPK; fPrPK=0; }
    if(fAPK) { delete fAPK; fAPK=0; }
    if(fPJT) { delete fPJT; fPJT=0; }
    if(fPJR) { delete fPJR; fPJR=0; }
    if(fPdJT) { delete fPdJT; fPdJT=0; }
    if(fPdJR) { delete fPdJR; fPdJR=0; }
    if(fPjT) { delete fPjT; fPjT=0; }
    if(fPjR) { delete fPjR; fPjR=0; }
    if(fPdjT) { delete fPdjT; fPdjT=0; }
    if(fPdjR) { delete fPdjR; fPdjR=0; }
  }

  void RigidBody::updateh(double t, int j) {

    Vec3 WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity() - m*frame[0]->getGyroscopicAccelerationOfTranslation(j);
    Vec3 WM = crossProduct(WThetaS*frame[0]->getAngularVelocity(),frame[0]->getAngularVelocity()) - WThetaS*frame[0]->getGyroscopicAccelerationOfRotation(j);

    h[j] += frame[0]->getJacobianOfTranslation(j).T()*WF + frame[0]->getJacobianOfRotation(j).T()*WM;
  }

  void RigidBody::updateh0Fromh1(double t) {
    h[0] += frame[0]->getJacobianOfTranslation(0).T()*(h[1](0,2) - m*frame[0]->getGyroscopicAccelerationOfTranslation()) + frame[0]->getJacobianOfRotation(0).T()*(h[1](3,5) - WThetaS*frame[0]->getGyroscopicAccelerationOfRotation());
  }

  void RigidBody::updateW0FromW1(double t) {
    W[0] += frame[0]->getJacobianOfTranslation(0).T()*W[1](Index(0,2),Index(0,W[1].cols()-1)) + frame[0]->getJacobianOfRotation(0).T()*W[1](Index(3,5),Index(0,W[1].cols()-1));
  }

  void RigidBody::updateV0FromV1(double t) {
    V[0] += frame[0]->getJacobianOfTranslation(0).T()*V[1](Index(0,2),Index(0,V[1].cols()-1)) + frame[0]->getJacobianOfRotation(0).T()*V[1](Index(3,5),Index(0,V[1].cols()-1));
  }

  void RigidBody::updatehInverseKinetics(double t, int j) {
    VecV buf = frame[0]->getJacobianOfTranslation(j).T()*(m*(frame[0]->getJacobianOfTranslation()*udall[0] + frame[0]->getGyroscopicAccelerationOfTranslation())) + frame[0]->getJacobianOfRotation(j).T()*(WThetaS*(frame[0]->getJacobianOfRotation()*udall[0] + frame[0]->getGyroscopicAccelerationOfRotation()));
    h[j] -= buf;
  }

  void RigidBody::updateStateDerivativeDependentVariables(double t) {
    for(unsigned int i=0; i<frame.size(); i++) {
      frame[i]->setAcceleration(frame[i]->getJacobianOfTranslation()*udall[0] + frame[i]->getGyroscopicAccelerationOfTranslation());
      frame[i]->setAngularAcceleration(frame[i]->getJacobianOfRotation()*udall[0] + frame[i]->getGyroscopicAccelerationOfRotation());
    }
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setReferenceAcceleration(contour[i]->getReferenceJacobianOfTranslation()*udall[0] + contour[i]->getReferenceGyroscopicAccelerationOfTranslation());
      contour[i]->setReferenceAngularAcceleration(contour[i]->getReferenceJacobianOfRotation()*udall[0] + contour[i]->getReferenceGyroscopicAccelerationOfRotation());
    }
  }

  void RigidBody::calcqSize() {
    Body::calcqSize();
    int nqT=0, nqR=0;
    nq = 0;
    if(dynamic_cast<LinearTranslation*>(fPrPK))
      nqT = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
    else if(fPrPK)
      nq = fPrPK->getqSize();
    if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
      nqR = 1;
    }
    else if(dynamic_cast<RotationAboutTwoAxes*>(fAPK)) {
      nqR = 2;
    }
    else if(dynamic_cast<RotationAboutThreeAxes*>(fAPK)) {
      if (not (dynamic_cast<TimeDependentCardanAngles*>(fAPK)))
        nqR = 3;
    }
    else if(fAPK) {
      int nqtmp = fAPK->getqSize();
      if(nq) assert(nq==nqtmp);
      nq = nqtmp;
    }
    nq += nqT + nqR;
    qSize = constraint ? 0 : nq;
  }

  void RigidBody::calcuSize(int j) {
    Body::calcuSize(j);
    int nuT=0, nuR=0;
    if(j==0) {
      nu[0] = 0;
      if(fPJT==0) {
        if(dynamic_cast<LinearTranslation*>(fPrPK))
          nuT = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
      } else
        nu[0] = fPJT->getuSize();

      if(fPJR==0) {
	if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
	  nuR = 1;
	}
	else if(dynamic_cast<RotationAboutTwoAxes*>(fAPK)) {
	  nuR = 2;
	}
	else if(dynamic_cast<RotationAboutThreeAxes*>(fAPK)) {
	  if (not (dynamic_cast<TimeDependentCardanAngles*>(fAPK)))
	    nuR = 3;
	}
      } else {
        int nutmp = fPJR->getuSize();
        if(nu[0]) assert(nu[0]==nutmp);
        nu[0] = nutmp;
      }
      nu[0] += nuT + nuR;
      uSize[j] = constraint ? 0 : nu[j];
    } else {
      nu[j] = 6;
      uSize[j] = 6;
    }
  }

  void RigidBody::init(InitStage stage) {
    if(stage==preInit) {
      Body::init(stage);
      if(constraint)
        dependency.push_back(constraint);
    }
    else if(stage==relativeFrameContourLocation) {
      // This outer loop is nessesary because the frame hierarchy must not be in the correct order!
      for(size_t k=0; k<saved_refFrameF.size(); k++)
        for(size_t j=0; j<saved_refFrameF.size(); j++) {
          int i = 0;
          if(saved_refFrameF[j]!="") i = frameIndex(getFrame(saved_refFrameF[j]));

          SrSF[j+1]=SrSF[i] + ASF[i]*saved_RrRF[j];
          ASF[j+1]=ASF[i]*saved_ARF[j];
        }
      for(size_t j=0; j<saved_refFrameC.size(); j++) {
        int i = 0;
        if(saved_refFrameC[j]!="") i = frameIndex(getFrame(saved_refFrameC[j]));

        SrSC[j]=SrSF[i] + ASF[i]*saved_RrRC[j];
        ASC[j]=ASF[i]*saved_ARC[j];
      }
    }
    else if(stage==resize) {
      if(iKinematics == -1)
        iKinematics = 0;

      Body::init(stage);

      PJT[0].resize(nu[0]);
      PJR[0].resize(nu[0]);

      PdJT.resize(nu[0]);
      PdJR.resize(nu[0]);

      PJT[1].resize(nu[1]);
      PJR[1].resize(nu[1]);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;

      if(cb)
        PJR0.resize(nu[0]);

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

      frame[0]->getJacobianOfTranslation(1) = PJT[1];
      frame[0]->getJacobianOfRotation(1) = PJR[1];

      if(fPJT==0) {
        Mat3V JT;
        if(dynamic_cast<LinearTranslation*>(fPrPK)) {
          JT.assign(dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors());
        }
        PJT[0].set(Index(0,2), Index(0,JT.cols()-1),JT);
      }
      if(fPJR==0) {
        Mat3V JR;

        if(dynamic_cast<RotationAboutXAxis*>(fAPK))
          JR.assign(Vec3("[1;0;0]"));
	else if(dynamic_cast<RotationAboutYAxis*>(fAPK))
          JR.assign(Vec3("[0;1;0]"));
	else if(dynamic_cast<RotationAboutZAxis*>(fAPK))
          JR.assign(Vec3("[0;0;1]"));
        else if(dynamic_cast<RotationAboutFixedAxis*>(fAPK))
          JR.assign(dynamic_cast<RotationAboutFixedAxis*>(fAPK)->getAxisOfRotation());
        else if(dynamic_cast<RotationAboutAxesYZ*>(fAPK)) {
          fPJR = new JRotationAboutAxesYZ(nu[0]);
          fPdJR = new JdRotationAboutAxesYZ(nu[0]);
        }
        else if(dynamic_cast<RotationAboutAxesXY*>(fAPK)) {
          fPJR = new JRotationAboutAxesXY(nu[0]);
          fPdJR = new JdRotationAboutAxesXY(nu[0]);
        }
        else if(dynamic_cast<CardanAngles*>(fAPK)) {
          JR.assign(Mat33(EYE));
          if(cb)
            fT = new TCardanAngles2(nq,nu[0]);
          else
            fT = new TCardanAngles(nq,nu[0]);
        }
        else if(dynamic_cast<RotationAboutAxesXYZ*>(fAPK)) {
          fPJR = new JRotationAboutAxesXYZ(nu[0]);
          fPdJR = new JdRotationAboutAxesXYZ(nu[0]);
        }
	else if(dynamic_cast<EulerAngles*>(fAPK)) {
          JR.assign(Mat33(EYE));
          if(cb)
            fT = new TEulerAngles2(nq,nu[0]);
          else
            fT = new TEulerAngles(nq,nu[0]);
        }

        PJR[0].set(Index(0,2), Index(nu[0]-JR.cols(),nu[0]-1),JR);

        if(cb) {
          if(iKinematics == 0 && dynamic_cast<DynamicSystem*>(frameOfReference->getParent())) {
            updateM_ = &RigidBody::updateMConst;
            Mbuf = m*JTJ(PJT[0]) + JTMJ(SThetaS,PJR[0]);
            LLM[0] = facLL(Mbuf);
            facLLM_ = &RigidBody::facLLMConst;
          }
          PJR0 = PJR[0];
        }
      }

      if(iInertia != 0)
        SThetaS = JMJT(ASF[iInertia],SThetaS) - m*JTJ(tilde(SrSF[iInertia]));

      if(constraint)
	TRel.resize(nq,nu[0]);

      for(int i=0; i<nu[0]; i++)
        TRel(i,i) = 1;
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
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+frameOfReference->getParent()->getName()+"_"+name);
    ds->addInverseKineticsLink(joint);
    joint->setForceDirection(Mat3V(3,EYE));
    joint->setMomentDirection(Mat3V(3,EYE));
    joint->connect(frameOfReference,frame[iKinematics]);
    joint->setBody(this);
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

  void RigidBody::updateKinematicsForSelectedFrame(double t) {
    if(fPJT)
      PJT[0] = (*fPJT)(qRel,t);
    if(fPJR)
      PJR[0] = (*fPJR)(qRel,t);

    if(fPjT)
      PjT = (*fPjT)(t);
    if(fPjR)
      PjR = (*fPjR)(t);

    if(fAPK)
      APK = (*fAPK)(qRel,t);
    if(fPrPK)
      PrPK = (*fPrPK)(qRel,t);

    frame[iKinematics]->setOrientation(frameOfReference->getOrientation()*APK);

    if(cb)
      PJR[0] = frameOfReference->getOrientation().T()*frame[iKinematics]->getOrientation()*PJR0;

    WrPK = frameOfReference->getOrientation()*PrPK;
    WomPK = frameOfReference->getOrientation()*(PJR[0]*uRel + PjR);
    WvPKrel = frameOfReference->getOrientation()*(PJT[0]*uRel + PjT);

    frame[iKinematics]->setAngularVelocity(frameOfReference->getAngularVelocity() + WomPK);
    frame[iKinematics]->setPosition(WrPK + frameOfReference->getPosition());
    frame[iKinematics]->setVelocity(frameOfReference->getVelocity() + WvPKrel + crossProduct(frameOfReference->getAngularVelocity(),WrPK));
  }

  void RigidBody::updateJacobiansForSelectedFrame0(double t) {
    frame[iKinematics]->getJacobianOfTranslation().init(0);
    frame[iKinematics]->getJacobianOfRotation().init(0);

    if(fPdJT)
      PdJT = (*fPdJT)(TRel*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(TRel*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    SqrMat3 tWrPK = tilde(WrPK);
    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*uRel + PdjT + PJT[0]*jRel) + crossProduct(frameOfReference->getAngularVelocity(), 2.*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*uRel + PdjR + PJR[0]*jRel) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation().set(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1), frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation());
    frame[iKinematics]->getJacobianOfRotation().set(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1), frameOfReference->getJacobianOfRotation());

    frame[iKinematics]->getJacobianOfTranslation().add(Index(0,2),Index(0,gethSize(0)-1), frameOfReference->getOrientation()*PJT[0]*JRel[0]);
    frame[iKinematics]->getJacobianOfRotation().add(Index(0,2),Index(0,gethSize(0)-1), frameOfReference->getOrientation()*PJR[0]*JRel[0]);
  }

  void RigidBody::updateKinematicsForRemainingFramesAndContours(double t) {
    if(iKinematics != 0) // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      frame[0]->setOrientation(frame[iKinematics]->getOrientation()*ASF[iKinematics].T());

    for(unsigned int i=1; i<frame.size(); i++) {
      WrSF[i] = frame[0]->getOrientation()*SrSF[i];
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      WrSC[i] = frame[0]->getOrientation()*SrSC[i];
    }

    if(iKinematics != 0) { // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      frame[0]->setPosition(frame[iKinematics]->getPosition() - WrSF[iKinematics]);
      frame[0]->setVelocity(frame[iKinematics]->getVelocity() - crossProduct(frame[iKinematics]->getAngularVelocity(), WrSF[iKinematics]));
      frame[0]->setAngularVelocity(frame[iKinematics]->getAngularVelocity());
    }

    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
        frame[i]->setPosition(frame[0]->getPosition() + WrSF[i]);
        frame[i]->setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(), WrSF[i]));
        frame[i]->setAngularVelocity(frame[0]->getAngularVelocity());
        frame[i]->setOrientation(frame[0]->getOrientation()*ASF[i]);
      }
    }
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setReferenceOrientation(frame[0]->getOrientation()*ASC[i]);
      contour[i]->setReferenceAngularVelocity(frame[0]->getAngularVelocity());
      contour[i]->setReferencePosition(frame[0]->getPosition() + WrSC[i]);
      contour[i]->setReferenceVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(), WrSC[i]));
    }
    WThetaS = JTMJ(SThetaS,frame[0]->getOrientation().T());
  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours(double t, int j) {

    if(iKinematics != 0) { // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      SqrMat3 tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation(j) + tWrSK*frame[iKinematics]->getJacobianOfRotation(j),j);
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation(j),j);
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation(j) + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[iKinematics]->getAngularVelocity(),crossProduct(frame[iKinematics]->getAngularVelocity(),-WrSF[iKinematics])),j);
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation(j),j);
    }

    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
        SqrMat3 tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSK*frame[0]->getJacobianOfRotation(j),j);
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i])),j);
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);
      }
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat3 tWrSC = tilde(WrSC[i]);
      contour[i]->setReferenceJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);
      contour[i]->setReferenceJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSC*frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])),j);
    }

  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours1(double t) {
    int j = 1;

    for(unsigned int i=1; i<frame.size(); i++) {
      SqrMat3 tWrSK = tilde(WrSF[i]);
      frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSK*frame[0]->getJacobianOfRotation(j),j);
      frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      frame[i]->setGyroscopicAccelerationOfTranslation(crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i]))- tWrSK*frame[0]->getGyroscopicAccelerationOfRotation(j),j);
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat3 tWrSC = tilde(WrSC[i]);
      contour[i]->setReferenceJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSC*frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfTranslation(crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])) - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation(j),j);
    }
  }

  void RigidBody::updateqRef(const Vec& ref) {
    Object::updateqRef(ref);
    if(!constraint) qRel>>q;
  }

  void RigidBody::updateuRef(const Vec& ref) {
    Object::updateuRef(ref);
    if(!constraint) uRel>>u;
  }

  void RigidBody::updateTRef(const Mat& ref) {
    Object::updateTRef(ref);
    if(!constraint) TRel>>T;
  }

  void RigidBody::addFrame(Frame *cosy, const Vec3 &RrRF, const SqrMat3 &ARF, const string& refFrameName) {
    Body::addFrame(cosy);

    saved_refFrameF.push_back(refFrameName);
    saved_RrRF.push_back(RrRF); 
    saved_ARF.push_back(ARF);
    SrSF.push_back(Vec3());
    WrSF.push_back(Vec3());
    ASF.push_back(SqrMat3());
  }

  void RigidBody::addFrame(Frame *frame_, const fmatvec::Vec3 &RrRF, const fmatvec::SqrMat3 &ARF, const Frame* refFrame) {
    addFrame(frame_, RrRF, ARF, refFrame?refFrame->getName():"C");
  }

  void RigidBody::addFrame(const string &str, const Vec3 &SrSF, const SqrMat3 &ASF, const Frame* refFrame) {
    addFrame(new Frame(str),SrSF,ASF,refFrame);
  }

  void RigidBody::addContour(Contour* contour, const Vec3 &RrRC, const SqrMat3 &ARC, const string& refFrameName) {
    Body::addContour(contour);

    saved_refFrameC.push_back(refFrameName);
    saved_RrRC.push_back(RrRC);
    saved_ARC.push_back(ARC);
    SrSC.push_back(Vec3());
    WrSC.push_back(Vec3());
    ASC.push_back(SqrMat3());
  }

  void RigidBody::addContour(Contour* contour, const fmatvec::Vec3 &RrRC, const fmatvec::SqrMat3 &ARC, const Frame* refFrame) {
    addContour(contour, RrRC, ARC, refFrame?refFrame->getName():"C");
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void RigidBody::setOpenMBVRigidBody(OpenMBV::RigidBody* body) {
    openMBVBody=body;
  }
#endif

  void RigidBody::updateMConst(double t, int i) {
    M[i] += Mbuf; // TODO
  }

  void RigidBody::updateMNotConst(double t, int i) {
    M[i] += m*JTJ(frame[0]->getJacobianOfTranslation(i)) + JTMJ(WThetaS,frame[0]->getJacobianOfRotation(i));
  }

  void RigidBody::updatePositionAndOrientationOfFrame(double t, unsigned int i) {

    if(fAPK)
      APK = (*fAPK)(qRel,t);
    if(fPrPK)
      PrPK = (*fPrPK)(qRel,t);

    frame[iKinematics]->setOrientation(frameOfReference->getOrientation()*APK);

    WrPK = frameOfReference->getOrientation()*PrPK;

    frame[iKinematics]->setPosition(WrPK + frameOfReference->getPosition());

    if(iKinematics != 0) // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      frame[0]->setOrientation(frame[iKinematics]->getOrientation()*ASF[iKinematics].T());

    WrSF[iKinematics] = frame[0]->getOrientation()*SrSF[iKinematics]; // TODO prüfen ob immer nötig
    if(i>0)
      WrSF[i] = frame[0]->getOrientation()*SrSF[i];


    if(iKinematics != 0) { // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      frame[0]->setPosition(frame[iKinematics]->getPosition() - WrSF[iKinematics]);
    }

    if(i>0) {
      if(i!=unsigned(iKinematics)) {
        frame[i]->setPosition(frame[0]->getPosition() + WrSF[i]);
        frame[i]->setOrientation(frame[0]->getOrientation()*ASF[i]);
      }
    }
  }

  void RigidBody::updateRelativeJacobians(double t, unsigned int i) {

    if(fPJT)
      PJT[0] = (*fPJT)(qRel,t);
    if(fPJR)
      PJR[0] = (*fPJR)(qRel,t);

    if(fPjT)
      PjT = (*fPjT)(t);
    if(fPjR)
      PjR = (*fPjR)(t);

    WJRrel = frameOfReference->getOrientation()*PJR[0];
    WJTrel = frameOfReference->getOrientation()*PJT[0];

    frame[iKinematics]->setVelocity(frameOfReference->getOrientation()*PjT+frameOfReference->getVelocity() + crossProduct(frameOfReference->getAngularVelocity(),WrPK));
    frame[iKinematics]->setAngularVelocity(frameOfReference->getOrientation()*PjR + frameOfReference->getAngularVelocity());

    if(iKinematics != 0) {
      frame[0]->setVelocity(frame[iKinematics]->getVelocity() - crossProduct(frame[iKinematics]->getAngularVelocity(), WrSF[iKinematics]));
      frame[0]->setAngularVelocity(frame[iKinematics]->getAngularVelocity());

      WJTrel += tilde(WrSF[iKinematics])*WJRrel;
    }

    if(i>0) {
      if(i!=unsigned(iKinematics)) {
        frame[i]->setAngularVelocity(frame[0]->getAngularVelocity());
        frame[i]->setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(), WrSF[i]));
        WJTrel -= tilde(WrSF[i])*WJRrel;

      }
    }
  }

  void RigidBody::updateAccelerations(double t, unsigned int i) {
    frame[iKinematics]->getJacobianOfTranslation().init(0);
    frame[iKinematics]->getJacobianOfRotation().init(0);
    if(fPdJT)
      PdJT = (*fPdJT)(TRel*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(TRel*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    WomPK = frameOfReference->getOrientation()*(PJR[0]*uRel + PjR);
    WvPKrel = frameOfReference->getOrientation()*(PJT[0]*uRel + PjT);
    frame[i]->setAngularVelocity(frameOfReference->getAngularVelocity() + WomPK);

    // TODO prüfen ob Optimierungspotential

    SqrMat3 tWrPK = tilde(WrPK);

    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*uRel + PdjT) + crossProduct(frameOfReference->getAngularVelocity(), 2.*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*uRel + PdjR) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation().set(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1), frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation());
    frame[iKinematics]->getJacobianOfRotation().set(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1), frameOfReference->getJacobianOfRotation());

    if(iKinematics != 0) {

      SqrMat3 tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation() + tWrSK*frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation() + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[i]->getAngularVelocity(),crossProduct(frame[i]->getAngularVelocity(),-WrSF[iKinematics]))); // frame[iKinematics]->getAngularVelocity() not up to date
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation());
    }

    if(i>0) {
      if(i!=unsigned(iKinematics)) {
        SqrMat3 tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrSK*frame[0]->getJacobianOfRotation());
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation());
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[i]->getAngularVelocity(),crossProduct(frame[i]->getAngularVelocity(),WrSF[i])));// frame[0]->getAngularVelocity() not up to date
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
      }
    }

  }

  void RigidBody::updateRelativeJacobians(double t, unsigned int i, Mat3V &WJTrel0, Mat3V &WJRrel0) {

    if(iKinematics != 0) {
      WJTrel0 += tilde(WrSF[iKinematics])*WJRrel0;
    }

    // TODO: Zusammenfassen
    if(i>0) {
      if(i!=unsigned(iKinematics)) {
        WJTrel0 -= tilde(WrSF[i])*WJRrel0;
      }
    }
  }

  void RigidBody::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMNS"frame") {
      TiXmlElement *ec=e->FirstChildElement();
      Frame *f=new Frame(ec->Attribute("name"));
      f->initializeUsingXML(ec);
#ifdef HAVE_OPENMBVCPPINTERFACE
      TiXmlElement *ee;
      if((ee=ec->FirstChildElement(MBSIMNS"enableOpenMBV")))
        f->enableOpenMBV(getDouble(ee->FirstChildElement(MBSIMNS"size")),
            getDouble(ee->FirstChildElement(MBSIMNS"offset")));
#endif
      ec=ec->NextSiblingElement();
      string refF="C";
      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
        refF=ec->Attribute("ref");
        refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
        ec=ec->NextSiblingElement();
      }
      Vec3 RrRF=getVec3(ec);
      ec=ec->NextSiblingElement();
      SqrMat3 ARF=getSqrMat3(ec);
      addFrame(f, RrRF, ARF, refF);
      e=e->NextSiblingElement();
    }

    // contours
    e=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMNS"contour") {
      TiXmlElement *ec=e->FirstChildElement();
      Contour *c=ObjectFactory::getInstance()->createContour(ec);
      TiXmlElement *contourElement=ec; // save for later initialization
      ec=ec->NextSiblingElement();
      string refF="C";
      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
        refF=ec->Attribute("ref");
        refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
        ec=ec->NextSiblingElement();
      }
      Vec3 RrRC=getVec3(ec);
      ec=ec->NextSiblingElement();
      SqrMat3 ARC=getSqrMat3(ec);
      addContour(c, RrRC, ARC, refF);
      c->initializeUsingXML(contourElement);
      e=e->NextSiblingElement();
    }

    e=element->FirstChildElement(MBSIMNS"frameForKinematics");
    setFrameForKinematics(getByPath<Frame>(e->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=element->FirstChildElement(MBSIMNS"mass");
    setMass(getDouble(e));
    e=element->FirstChildElement(MBSIMNS"inertiaTensor");
    setInertiaTensor(getSymMat3(e));
    e=element->FirstChildElement(MBSIMNS"translation");
    Translation *trans=ObjectFactory::getInstance()->createTranslation(e->FirstChildElement());
    if(trans) {
      setTranslation(trans);
      trans->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"rotation");
    Rotation *rot=ObjectFactory::getInstance()->createRotation(e->FirstChildElement());
    if(rot) {
      setRotation(rot);
      rot->initializeUsingXML(e->FirstChildElement());
    }
    // BEGIN The following elements are rarly used. That is why they are optional
    e=element->FirstChildElement(MBSIMNS"jacobianOfTranslation");
    if(e) {
      Jacobian *jac=ObjectFactory::getInstance()->createJacobian(e->FirstChildElement());
      setJacobianOfTranslation(jac);
      jac->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"jacobianOfRotation");
    if(e) {
      Jacobian *jac=ObjectFactory::getInstance()->createJacobian(e->FirstChildElement());
      setJacobianOfRotation(jac);
      jac->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfJacobianOfTranslation");
    if(e) {
      Function3<Mat,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
      setDerivativeOfJacobianOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfJacobianOfRotation");
    if(e) {
      Function3<Mat,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
      setDerivativeOfJacobianOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfTranslation");
    if(e) {
      Function1<Vec,double> *f=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
      setGuidingVelocityOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfRotation");
    if(e) {
      Function1<Vec,double> *f=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
      setGuidingVelocityOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfTranslation");
    if(e) {
      Function1<Vec,double> *f=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
      setDerivativeOfGuidingVelocityOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfRotation");
    if(e) {
      Function1<Vec,double> *f=ObjectFactory::getInstance()->createFunction1_VS(e->FirstChildElement());
      setDerivativeOfGuidingVelocityOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    // END
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
    if(e) {
      OpenMBV::RigidBody *rb=dynamic_cast<OpenMBV::RigidBody*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
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
    }
#endif
  }

}
