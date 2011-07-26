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
 * Contact: mfoerg@users.berlios.de
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
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), m(0), SThetaS(3,INIT,0.), WThetaS(3,INIT,0.), iKinematics(-1), iInertia(-1), cb(false), PjT(3,INIT,0.), PjR(3,INIT,0.), PdjT(3,INIT,0.), PdjR(3,INIT,0.), APK(3,INIT,0.), PrPK(3,INIT,0.), WrPK(3,INIT,0.), WvPKrel(3,INIT,0.), WomPK(3,INIT,0.), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0), constraint(0) {
    APK(0,0)=1.;
    APK(1,1)=1.;
    APK(2,2)=1.;

    C=new Frame("C");
    Body::addFrame(C);
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVFrame=C;
#endif

    SrSF.push_back(Vec(3,INIT,0.));
    WrSF.push_back(Vec(3,INIT,0.));
    ASF.push_back(SqrMat(3,EYE));

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

    Vec WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity() - m*frame[0]->getGyroscopicAccelerationOfTranslation(j);
    Vec WM = crossProduct(WThetaS*frame[0]->getAngularVelocity(),frame[0]->getAngularVelocity()) - WThetaS*frame[0]->getGyroscopicAccelerationOfRotation(j);

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
    h[j] -= frame[0]->getJacobianOfTranslation(j).T()*m*(frame[0]->getJacobianOfTranslation()*udall[0] + frame[0]->getGyroscopicAccelerationOfTranslation()) + frame[0]->getJacobianOfRotation(j).T()*WThetaS*(frame[0]->getJacobianOfRotation()*udall[0] + frame[0]->getGyroscopicAccelerationOfRotation());
  }

  void RigidBody::updateStateDerivativeDependentVariables(double t) {

    aT = frame[0]->getJacobianOfTranslation()*udall[0];// + frame[0]->getGyroscopicAccelerationOfTranslation();
    aR = frame[0]->getJacobianOfRotation()*udall[0]; //+ frame[0]->getGyroscopicAccelerationOfRotation(); 
  }

  void RigidBody::calcqSize() {
    Body::calcqSize();
    int nqT=0, nqR=0;
    if(dynamic_cast<LinearTranslation*>(fPrPK)) {
      nqT += dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
      nqR = nqT;
    }
    else if(fPrPK)
      nqT = fPrPK->getqSize();
    if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
      nqR += 1; 
      nqT = nqR;
    }
    else if(fAPK)
      nqR = fAPK->getqSize();
    // TODO: besseres Konzept überlegen
    assert(nqT == nqR);
    nq = nqT;
    qSize = constraint ? 0 : nq;
  }

  void RigidBody::calcuSize(int j) {
    Body::calcuSize(j);
    int nuT=0, nuR=0;
    if(j==0) {
      if(fPJT==0) {
	if(dynamic_cast<LinearTranslation*>(fPrPK)) {
	  nuT += dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors().cols();
	  nuR = nuT;
	} else
	  nuT = 0;
      } 

      if(fPJR==0) {
	if(dynamic_cast<RotationAboutOneAxis*>(fAPK)) {
	  nuR += 1; 
	  nuT = nuR;
	} 
      } else
        nuR = fPJR->getuSize();
      assert(nuT == nuR);
      nu[j] = nuT;
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

      PJT[0].resize(3,nu[0]);
      PJR[0].resize(3,nu[0]);

      PdJT.resize(3,nu[0]);
      PdJR.resize(3,nu[0]);

      PJT[1].resize(3,nu[1]);
      PJR[1].resize(3,nu[1]);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;

      JRel.resize(nu[0],hSize[0]);
      for(int i=0; i<uSize[0]; i++)
        JRel(i,hSize[0]-uSize[0]+i) = 1;
      jRel.resize(nu[0]);
      qRel.resize(nq);
      uRel.resize(nu[0]);
      q.resize(qSize);
      u.resize(uSize[0]);

      WJTrel.resize(3,nu[0]);
      WjTrel.resize(3);
      WJRrel.resize(3,nu[0]);
      WjRrel.resize(3);

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;
    }
    else if(stage==MBSim::unknownStage) {
      Body::init(stage);

  frame[0]->getJacobianOfTranslation(1) = PJT[1];
  frame[0]->getJacobianOfRotation(1) = PJR[1];

      if(fPJT==0) {
        Mat JT(3,0);
        if(dynamic_cast<LinearTranslation*>(fPrPK)) {
          JT.resize() = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors();
        } 
        PJT[0](Index(0,2), Index(0,JT.cols()-1)) = JT;
        //PJT[1](Index(0,2), Index(0,JT.cols()-1)) = JT;
        //PJT[1](Index(0,2), Index(JT.cols(),JT.cols()+forceDir.cols()-1)) = forceDir;
      }
      if(fPJR==0) {
        Mat JR(3,0);

        if(dynamic_cast<RotationAboutXAxis*>(fAPK)) 
          JR.resize() = Vec("[1;0;0]");
	else if(dynamic_cast<RotationAboutYAxis*>(fAPK)) 
          JR.resize() = Vec("[0;1;0]");
	else if(dynamic_cast<RotationAboutZAxis*>(fAPK)) 
          JR.resize() = Vec("[0;0;1]");
        else if(dynamic_cast<RotationAboutFixedAxis*>(fAPK)) 
          JR.resize() = dynamic_cast<RotationAboutFixedAxis*>(fAPK)->getAxisOfRotation();
        else if(dynamic_cast<RotationAboutAxesYZ*>(fAPK)) {
          fPJR = new JRotationAboutAxesYZ(nu[0]);
          fPdJR = new JdRotationAboutAxesYZ(nu[0]);
        }
        else if(dynamic_cast<RotationAboutAxesXY*>(fAPK)) {
          fPJR = new JRotationAboutAxesXY(nu[0]);
          fPdJR = new JdRotationAboutAxesXY(nu[0]);
        }
        else if(dynamic_cast<CardanAngles*>(fAPK)) {
          JR.resize() << DiagMat(3,INIT,1);
          if(cb) {
            fT = new TCardanAngles2(nq,nu[0]);
          }
          else {
            fT = new TCardanAngles(nq,nu[0]);
          }
        }
	else if(dynamic_cast<EulerAngles*>(fAPK)) {
          JR.resize() << DiagMat(3,INIT,1);
          if(cb) {
            fT = new TEulerAngles2(nq,nu[0]);
          }
          else {
            fT = new TEulerAngles(nq,nu[0]);
          }
        }

        PJR[0](Index(0,2), Index(nu[0]-JR.cols(),nu[0]-1)) = JR;
       // PJR[1](Index(0,2), Index(nu[1]-momentDir.cols()-JR.cols(),nu[1]-momentDir.cols()-1)) = JR;
       // PJR[1](Index(0,2), Index(nu[1]-momentDir.cols(),nu[1]-1)) = momentDir;

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
        SThetaS = SymMat(ASF[iInertia]*SThetaS*ASF[iInertia].T()) - m*JTJ(tilde(SrSF[iInertia]));

      if(constraint)
        T.resize(nq,nu[0]);

      for(int i=0; i<nu[0]; i++) 
        T(i,i) = 1;
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(notMinimalState)==enabled) {
          for(int i=0; i<nq; i++)
            plotColumns.push_back("qRel("+numtostr(i)+")");
          for(int i=0; i<nu[0]; i++)
            plotColumns.push_back("uRel("+numtostr(i)+")");
        }
        if(getPlotFeature(globalPosition)==enabled) {
          plotColumns.push_back("WxOS");
          plotColumns.push_back("WyOS");
          plotColumns.push_back("WzOS");
          plotColumns.push_back("alpha");
          plotColumns.push_back("beta");
          plotColumns.push_back("gamma");
        }
        if(getPlotFeature(globalVelocity)==enabled) {
          plotColumns.push_back("WvxS");
          plotColumns.push_back("WvyS");
          plotColumns.push_back("WvzS");
          plotColumns.push_back("alphap");
          plotColumns.push_back("betap");
          plotColumns.push_back("gammap");
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
    MyJoint *joint = new MyJoint(string("Joint_")+name);
    ds->addInverseKineticsLink(joint);
    joint->setForceDirection(SqrMat(3,EYE));
    joint->setJacobianOfTranslation(fPJT);
    joint->setTranslation(fPrPK);
    joint->setMomentDirection(SqrMat(3,EYE));
    joint->setJacobianOfRotation(fPJR);
    joint->setRotation(fAPK);
    joint->connect(frameOfReference,frame[iKinematics]);
  }

  void RigidBody::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(notMinimalState)==enabled) {
        for(int i=0; i<nq; i++)
          plotVector.push_back(qRel(i));
        for(int i=0; i<nu[0]; i++)
          plotVector.push_back(uRel(i));
      }
      if(getPlotFeature(globalPosition)==enabled) {
        Vec WrOS=frame[0]->getPosition();
        Vec cardan=AIK2Cardan(frame[0]->getOrientation());
        plotVector.push_back(WrOS(0));
        plotVector.push_back(WrOS(1));
        plotVector.push_back(WrOS(2));
        plotVector.push_back(cardan(0));
        plotVector.push_back(cardan(1));
        plotVector.push_back(cardan(2));
      }
      if(getPlotFeature(globalVelocity)==enabled) {
        Vec WvS    =frame[0]->getVelocity();
        Vec WomegaS=frame[0]->getAngularVelocity();
        plotVector.push_back(WvS(0));
        plotVector.push_back(WvS(1));
        plotVector.push_back(WvS(2));
        plotVector.push_back(WomegaS(0));
        plotVector.push_back(WomegaS(1));
        plotVector.push_back(WomegaS(2));
      }

#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        Vec WrOS=openMBVFrame->getPosition();
        Vec cardan=AIK2Cardan(openMBVFrame->getOrientation());
        data.push_back(WrOS(0));
        data.push_back(WrOS(1));
        data.push_back(WrOS(2));
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        ((OpenMBV::RigidBody*)openMBVBody)->append(data);
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

    if(cb) {
      PJR[0] = frameOfReference->getOrientation().T()*frame[iKinematics]->getOrientation()*PJR0;
    }

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
      PdJT = (*fPdJT)(T*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(T*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    SqrMat tWrPK = tilde(WrPK);
    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*uRel + PdjT + PJT[0]*jRel) + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*uRel + PdjR + PJR[0]*jRel) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,gethSize(0)-1)) += frameOfReference->getOrientation()*PJT[0]*JRel;
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,gethSize(0)-1)) += frameOfReference->getOrientation()*PJR[0]*JRel;
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
      SqrMat tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation(j) + tWrSK*frame[iKinematics]->getJacobianOfRotation(j),j);
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation(j),j);
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation(j) + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[iKinematics]->getAngularVelocity(),crossProduct(frame[iKinematics]->getAngularVelocity(),-WrSF[iKinematics])),j);
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation(j),j);
    }

    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
	SqrMat tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSK*frame[0]->getJacobianOfRotation(j),j);
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i])),j);
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);
      }
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setReferenceJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);
      contour[i]->setReferenceJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSC*frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])),j);
    }

  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours1(double t) {
    int j = 1;

    for(unsigned int i=1; i<frame.size(); i++) {
      SqrMat tWrSK = tilde(WrSF[i]);
      frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSK*frame[0]->getJacobianOfRotation(j),j);
      frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      frame[i]->setGyroscopicAccelerationOfTranslation( - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i])),j);
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setReferenceJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrSC*frame[0]->getJacobianOfRotation(j),j);
      contour[i]->setReferenceGyroscopicAccelerationOfTranslation( - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])),j);
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

  void RigidBody::addFrame(Frame *cosy, const Vec &RrRF, const SqrMat &ARF, const string& refFrameName) {
    Body::addFrame(cosy);

    saved_refFrameF.push_back(refFrameName);
    saved_RrRF.push_back(RrRF.copy()); // use .copy() because the copy constructor of fmatvec is a reference
    saved_ARF.push_back(ARF.copy()); // use .copy() because the copy constructor of fmatvec is a reference
    SrSF.push_back(Vec(3));
    WrSF.push_back(Vec(3));
    ASF.push_back(SqrMat(3));
  }

  void RigidBody::addFrame(Frame *frame_, const fmatvec::Vec &RrRF, const fmatvec::SqrMat &ARF, const Frame* refFrame) {
        addFrame(frame_, RrRF, ARF, refFrame?refFrame->getName():"C");
  }

  void RigidBody::addFrame(const string &str, const Vec &SrSF, const SqrMat &ASF, const Frame* refFrame) {
    addFrame(new Frame(str),SrSF,ASF,refFrame);
  }

  void RigidBody::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const string& refFrameName) {
    Body::addContour(contour);

    saved_refFrameC.push_back(refFrameName);
    saved_RrRC.push_back(RrRC.copy()); // use .copy() because the copy constructor of fmatvec is a reference
    saved_ARC.push_back(ARC.copy()); // use .copy() because the copy constructor of fmatvec is a reference
    SrSC.push_back(Vec(3));
    WrSC.push_back(Vec(3));
    ASC.push_back(SqrMat(3));
  }

  void RigidBody::addContour(Contour* contour, const fmatvec::Vec &RrRC, const fmatvec::SqrMat &ARC, const Frame* refFrame) {
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
      PdJT = (*fPdJT)(T*uRel,qRel,t);
    if(fPdJR)
      PdJR = (*fPdJR)(T*uRel,qRel,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    WomPK = frameOfReference->getOrientation()*(PJR[0]*uRel + PjR);
    WvPKrel = frameOfReference->getOrientation()*(PJT[0]*uRel + PjT);
    frame[i]->setAngularVelocity(frameOfReference->getAngularVelocity() + WomPK);

    // TODO prüfen ob Optimierungspotential

    SqrMat tWrPK = tilde(WrPK);

    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*uRel + PdjT) + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*uRel + PdjR) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();

    if(iKinematics != 0) { 

      SqrMat tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation() + tWrSK*frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation() + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[i]->getAngularVelocity(),crossProduct(frame[i]->getAngularVelocity(),-WrSF[iKinematics]))); // frame[iKinematics]->getAngularVelocity() not up to date
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation());
    }

    if(i>0) {
      if(i!=unsigned(iKinematics)) {
        SqrMat tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrSK*frame[0]->getJacobianOfRotation());
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation());
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[i]->getAngularVelocity(),crossProduct(frame[i]->getAngularVelocity(),WrSF[i])));// frame[0]->getAngularVelocity() not up to date
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
      }
    }

  }

  void RigidBody::updateRelativeJacobians(double t, unsigned int i, Mat &WJTrel0, Mat &WJRrel0) {

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
      Vec RrRF=getVec(ec,3);
      ec=ec->NextSiblingElement();
      SqrMat ARF=getSqrMat(ec,3);
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
      Vec RrRC=getVec(ec,3);
      ec=ec->NextSiblingElement();
      SqrMat ARC=getSqrMat(ec,3);
      addContour(c, RrRC, ARC, refF);
      c->initializeUsingXML(contourElement);
      e=e->NextSiblingElement();
    }

    e=element->FirstChildElement(MBSIMNS"frameForKinematics");
    setFrameForKinematics(getByPath<Frame>(e->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
    e=element->FirstChildElement(MBSIMNS"mass");
    setMass(getDouble(e));
    e=element->FirstChildElement(MBSIMNS"inertiaTensor");
    setInertiaTensor(getSymMat(e,3));
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
