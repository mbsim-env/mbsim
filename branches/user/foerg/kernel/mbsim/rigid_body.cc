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
#include "mbsim/contours/compound_contour.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), m(0), cb(false), APK(EYE), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0), constraint(0), frameForJacobianOfRotation(0) {

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
    W[0] += C->getJacobianOfTranslation(0).T()*W[1](Index(0,2),Index(0,W[1].cols()-1)) + C->getJacobianOfRotation(0).T()*W[1](Index(3,5),Index(0,W[1].cols()-1));
  }

  void RigidBody::updateV0FromV1(double t) {
    V[0] += C->getJacobianOfTranslation(0).T()*V[1](Index(0,2),Index(0,V[1].cols()-1)) + C->getJacobianOfRotation(0).T()*V[1](Index(3,5),Index(0,V[1].cols()-1));
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

      if(fPJT==0) {
        Mat3V JT;
        if(dynamic_cast<LinearTranslation*>(fPrPK)) {
          JT = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors();
        }
        else if(dynamic_cast<TimeDependentTranslation*>(fPrPK)) {
          DifferentiableFunction1<fmatvec::Vec3> *pos = dynamic_cast<DifferentiableFunction1<fmatvec::Vec3> *>(dynamic_cast<TimeDependentTranslation*>(fPrPK)->getTranslationFunction());
          if(pos) {
            if(fPjT==0) fPjT = &pos->getDerivative(1);
            if(fPdjT==0) fPdjT = &pos->getDerivative(2);
          }
        }
        PJT[0].set(Index(0,2), Index(0,JT.cols()-1),JT);
      }
      if(fPJR==0) {
        Mat3V JR;

        if(dynamic_cast<RotationAboutXAxis*>(fAPK))
          JR = Vec3("[1;0;0]");
	else if(dynamic_cast<RotationAboutYAxis*>(fAPK))
          JR = Vec3("[0;1;0]");
	else if(dynamic_cast<RotationAboutZAxis*>(fAPK))
          JR = Vec3("[0;0;1]");
        else if(dynamic_cast<RotationAboutFixedAxis*>(fAPK))
          JR = dynamic_cast<RotationAboutFixedAxis*>(fAPK)->getAxisOfRotation();
        else if(dynamic_cast<RotationAboutAxesYZ*>(fAPK)) {
          fPJR = new JRotationAboutAxesYZ(nu[0]);
          fPdJR = new JdRotationAboutAxesYZ(nu[0]);
        }
        else if(dynamic_cast<RotationAboutAxesXY*>(fAPK)) {
          fPJR = new JRotationAboutAxesXY(nu[0]);
          fPdJR = new JdRotationAboutAxesXY(nu[0]);
        }
        else if(dynamic_cast<CardanAngles*>(fAPK)) {
          JR = Mat33(EYE);
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
          JR = Mat33(EYE);
          if(cb)
            fT = new TEulerAngles2(nq,nu[0]);
          else
            fT = new TEulerAngles(nq,nu[0]);
        }

        PJR[0].set(Index(0,2), Index(nu[0]-JR.cols(),nu[0]-1),JR);

        // TODO: Alle Fälle überprüfen
        if(cb) {
          if(K != C && dynamic_cast<DynamicSystem*>(R->getParent())) {
            updateM_ = &RigidBody::updateMConst;
            Mbuf = m*JTJ(PJT[0]) + JTMJ(SThetaS,PJR[0]);
            LLM[0] = facLL(Mbuf);
            facLLM_ = &RigidBody::facLLMConst;
          }
        }
      }
      frameForJacobianOfRotation = cb?K:R;

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
    joint->setForceDirection(Mat3V(3,EYE));
    joint->setMomentDirection(Mat3V(3,EYE));
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

    K->setOrientation(R->getOrientation()*APK);

    WrPK = R->getOrientation()*PrPK;
    WomPK = frameForJacobianOfRotation->getOrientation()*(PJR[0]*uRel + PjR);
    WvPKrel = R->getOrientation()*(PJT[0]*uRel + PjT);

    K->setAngularVelocity(R->getAngularVelocity() + WomPK);
    K->setPosition(WrPK + R->getPosition());
    K->setVelocity(R->getVelocity() + WvPKrel + crossProduct(R->getAngularVelocity(),WrPK));
  }

  void RigidBody::updateJacobiansForSelectedFrame0(double t) {
    K->getJacobianOfTranslation().init(0);
    K->getJacobianOfRotation().init(0);

    if(fPdJT)
      PdJT = (*fPdJT)(TRel*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(TRel*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    SqrMat3 tWrPK = tilde(WrPK);
    K->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() - tWrPK*R->getGyroscopicAccelerationOfRotation() + R->getOrientation()*(PdJT*uRel + PdjT + PJT[0]*jRel) + crossProduct(R->getAngularVelocity(), 2.*WvPKrel+crossProduct(R->getAngularVelocity(),WrPK)));
    K->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->getOrientation()*(PdJR*uRel + PdjR + PJR[0]*jRel) + crossProduct(R->getAngularVelocity(), WomPK));

    K->getJacobianOfTranslation().set(Index(0,2),Index(0,R->getJacobianOfTranslation().cols()-1), R->getJacobianOfTranslation() - tWrPK*R->getJacobianOfRotation());
    K->getJacobianOfRotation().set(Index(0,2),Index(0,R->getJacobianOfRotation().cols()-1), R->getJacobianOfRotation());

    K->getJacobianOfTranslation().add(Index(0,2),Index(0,gethSize(0)-1), R->getOrientation()*PJT[0]*JRel[0]);
    K->getJacobianOfRotation().add(Index(0,2),Index(0,gethSize(0)-1), frameForJacobianOfRotation->getOrientation()*PJR[0]*JRel[0]);
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

  void RigidBody::updateTRef(const Mat& ref) {
    Object::updateTRef(ref);
    if(!constraint) TRel>>T;
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

  void RigidBody::addContour(Contour* contour_, const fmatvec::Vec3 &RrRC, const fmatvec::SqrMat3 &ARC, const Frame* refFrame) {
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
    M[i] += Mbuf; // TODO
  }

  void RigidBody::updateMNotConst(double t, int i) {
    M[i] += m*JTJ(C->getJacobianOfTranslation(i)) + JTMJ(WThetaS,C->getJacobianOfRotation(i));
  }

  void RigidBody::updatePositionAndOrientationOfFrame(double t, Frame *P) {

    if(fAPK)
      APK = (*fAPK)(qRel,t);
    if(fPrPK)
      PrPK = (*fPrPK)(qRel,t);

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

    if(fPJT)
      PJT[0] = (*fPJT)(qRel,t);
    if(fPJR)
      PJR[0] = (*fPJR)(qRel,t);

    if(fPjT)
      PjT = (*fPjT)(t);
    if(fPjR)
      PjR = (*fPjR)(t);

    WJRrel = frameForJacobianOfRotation->getOrientation()*PJR[0];
    WJTrel = R->getOrientation()*PJT[0];

    K->setVelocity(R->getOrientation()*PjT+R->getVelocity() + crossProduct(R->getAngularVelocity(),WrPK));
    K->setAngularVelocity(frameForJacobianOfRotation->getOrientation()*PjR + R->getAngularVelocity());

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
    if(fPdJT)
      PdJT = (*fPdJT)(TRel*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(TRel*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    WomPK = frameForJacobianOfRotation->getOrientation()*(PJR[0]*uRel + PjR);
    WvPKrel = R->getOrientation()*(PJT[0]*uRel + PjT);

    //frame[i]->setAngularVelocity(R->getAngularVelocity() + WomPK);

    // TODO prüfen ob Optimierungspotential

    SqrMat3 tWrPK = tilde(WrPK);

    K->setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() - tWrPK*R->getGyroscopicAccelerationOfRotation() + R->getOrientation()*(PdJT*uRel + PdjT) + crossProduct(R->getAngularVelocity(), 2.*WvPKrel+crossProduct(R->getAngularVelocity(),WrPK)));
    K->setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->getOrientation()*(PdJR*uRel + PdjR) + crossProduct(R->getAngularVelocity(), WomPK));

    K->getJacobianOfTranslation().set(Index(0,2),Index(0,R->getJacobianOfTranslation().cols()-1), R->getJacobianOfTranslation() - tWrPK*R->getJacobianOfRotation());
    K->getJacobianOfRotation().set(Index(0,2),Index(0,R->getJacobianOfRotation().cols()-1), R->getJacobianOfRotation());

   if(K != C) C->updateJacobians();
   if(P!=C && P!=K)
     ((FixedRelativeFrame*)P)->updateJacobians();
  }

  void RigidBody::updateRelativeJacobians(double t, Frame *P, Mat3V &WJTrel0, Mat3V &WJRrel0) {

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

      Contour *c=ObjectFactory::getInstance()->createContour(ec);
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
      Contour *c=ObjectFactory::getInstance()->createContour(e);
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
      Function3<Mat3V,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
      setDerivativeOfJacobianOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfJacobianOfRotation");
    if(e) {
      Function3<Mat3V,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
      setDerivativeOfJacobianOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfTranslation");
    if(e) {
      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
      setGuidingVelocityOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfRotation");
    if(e) {
      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
      setGuidingVelocityOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfTranslation");
    if(e) {
      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
      setDerivativeOfGuidingVelocityOfTranslation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }
    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfRotation");
    if(e) {
      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
      setDerivativeOfGuidingVelocityOfRotation(f);
      f->initializeUsingXML(e->FirstChildElement());
    }

    e=element->FirstChildElement(MBSIMNS"isFrameOfBodyForRotation");
    if(e)
      isFrameOfBodyForRotation(getBool(e));

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
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVWeightArrow(arrow);
    }

    e=element->FirstChildElement(MBSIMNS"openMBVJointForceArrow");
    if(e) {
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVJointForceArrow(arrow);
    }

    e=element->FirstChildElement(MBSIMNS"openMBVJointMomentArrow");
    if(e) {
      OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      if(!openMBVBody)
        setOpenMBVRigidBody(new OpenMBV::InvisibleBody);
      arrow->initializeUsingXML(e->FirstChildElement());
      setOpenMBVJointMomentArrow(arrow);
    }
#endif
  }

  TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Body::writeXMLFile(parent);
//
//    TiXmlElement * ele1 = new TiXmlElement( MBSIMNS"frameForKinematics" );
//    string str = string("Frame[") + getFrameForKinematics()->getName() + "]";
//    ele1->SetAttribute("ref", str);
//    ele0->LinkEndChild(ele1);
//
//    addElementText(ele0,MBSIMNS"mass",getMass());
//    addElementText(ele0,MBSIMNS"inertiaTensor",getInertiaTensor());
//
//    ele1 = new TiXmlElement( MBSIMNS"translation" );
//    if(getTranslation()) 
//      getTranslation()->writeXMLFile(ele1);
//    ele0->LinkEndChild(ele1);
//
//    ele1 = new TiXmlElement( MBSIMNS"rotation" );
//    if(getRotation()) 
//      getRotation()->writeXMLFile(ele1);
//    ele0->LinkEndChild(ele1);
//
//    ele1 = new TiXmlElement( MBSIMNS"frames" );
//    for(unsigned int i=1; i<frame.size(); i++) {
//      TiXmlElement* ele2 = new TiXmlElement( MBSIMNS"frame" );
//      ele1->LinkEndChild( ele2 );
//      frame[i]->writeXMLFile(ele2);
//      if(saved_refFrameF[i-1] != "C") {
//        TiXmlElement *ele3 = new TiXmlElement( MBSIMNS"frameOfReference" );
//        string str = string("Frame[") + saved_refFrameF[i-1] + "]";
//        ele3->SetAttribute("ref", str);
//        ele2->LinkEndChild(ele3);
//      }
//
//      addElementText(ele2,MBSIMNS"position",saved_RrRF[i-1]);
//      addElementText(ele2,MBSIMNS"orientation",saved_ARF[i-1]);
//    }
//    ele0->LinkEndChild( ele1 );
//
//    ele1 = new TiXmlElement( MBSIMNS"contours" );
//    for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
//      (*i)->writeXMLFile(ele1);
//    ele0->LinkEndChild( ele1 );
//
//#ifdef HAVE_OPENMBVCPPINTERFACE
//    if(getOpenMBVBody()) {
//      ele1 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
//      getOpenMBVBody()->writeXMLFile(ele1);
//
//      if(getOpenMBVFrameOfReference()) {
//        TiXmlElement * ele2 = new TiXmlElement( MBSIMNS"frameOfReference" );
//        string str = string("Frame[") + getOpenMBVFrameOfReference()->getName() + "]";
//        ele2->SetAttribute("ref", str);
//        ele1->LinkEndChild(ele2);
//      }
//      ele0->LinkEndChild(ele1);
//    }
//
//    if(C->getOpenMBVFrame()) {
//      ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
//      addElementText(ele1,MBSIMNS"size",C->getOpenMBVFrame()->getSize());
//      addElementText(ele1,MBSIMNS"offset",C->getOpenMBVFrame()->getOffset());
//      ele0->LinkEndChild(ele1);
//    }
//#endif

    return ele0;
  }


}
