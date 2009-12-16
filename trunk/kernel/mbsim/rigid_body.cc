/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include <mbsim/environment.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), m(0), SThetaS(3,INIT,0.), WThetaS(3,INIT,0.), iKinematics(-1), iInertia(-1), cb(false), PjT(3,INIT,0.), PjR(3,INIT,0.), PdjT(3,INIT,0.), PdjR(3,INIT,0.), APK(3,INIT,0.), PrPK(3,INIT,0.), WrPK(3,INIT,0.), WvPKrel(3,INIT,0.), WomPK(3,INIT,0.), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0) {
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

  void RigidBody::updateh(double t) {

    Vec WF = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity() - m*frame[0]->getGyroscopicAccelerationOfTranslation();
    Vec WM = crossProduct(WThetaS*frame[0]->getAngularVelocity(),frame[0]->getAngularVelocity()) - WThetaS*frame[0]->getGyroscopicAccelerationOfRotation();

    h += trans(frame[0]->getJacobianOfTranslation())*WF + trans(frame[0]->getJacobianOfRotation())*WM;
    hObject += trans(frame[0]->getJacobianOfTranslation())*WF + trans(frame[0]->getJacobianOfRotation())*WM;
  }

  void RigidBody::updatehInverseKinetics(double t) {

    h -= trans(frame[0]->getJacobianOfTranslation())*m*aT + trans(frame[0]->getJacobianOfRotation())*WThetaS*aR;
  }

  void RigidBody::updateStateDerivativeDependentVariables(double t) {

    aT = frame[0]->getJacobianOfTranslation()*udall;// + frame[0]->getGyroscopicAccelerationOfTranslation();
    aR = frame[0]->getJacobianOfRotation()*udall; //+ frame[0]->getGyroscopicAccelerationOfRotation(); 
  }

  void RigidBody::calcqSize() {
    Body::calcqSize();
    qSize = 0;
    if(fPrPK)
      qSize += fPrPK->getqSize();
    if(fAPK)
      qSize += fAPK->getqSize();
  }

  void RigidBody::calcuSize(int j) {
    Body::calcuSize(j);
    if(j==0) {
      uSize[j] = 0;
      if(fPJT==0) {
        LinearTranslation* fPrPK_ = dynamic_cast<LinearTranslation*>(fPrPK);
        if(fPrPK_) 
          uSize[j] += fPrPK->getqSize();
      } else
        uSize[j] += fPJT->getuSize();
      if(fPJR==0) {
        //TODO  Euler-Parameter
        //RotationAxis* fAPK_ = dynamic_cast<RotationAxis*>(fAPK);
        if(fAPK)
          uSize[j] += fAPK->getqSize();
        //CardanAngles* fAPK_ = dynamic_cast<CardanAngles*>(fAPK);
        //if(fAPK_)
        //uSize += fAPK->getqSize();
      } else
        uSize[j] += fPJR->getuSize();
    } else {
      uSize[j] = uSize[0];
      uSize[j] += forceDir.cols();
      uSize[j] += momentDir.cols();
    }
  }

  void RigidBody::init(InitStage stage) {
    if(stage==relativeFrameContourLocation) {
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
    else if(stage==unknownStage) {
      if(iKinematics == -1)
        iKinematics = 0;

      Body::init(stage);

      PJT.resize(3,uSize[0]);
      PJR.resize(3,uSize[0]);

      PdJT.resize(3,uSize[0]);
      PdJR.resize(3,uSize[0]);

      PJTs.resize(3,uSize[1]);
      PJRs.resize(3,uSize[1]);

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;

      if(fPJT==0) {
        Mat JT(3,0);
        if(dynamic_cast<LinearTranslation*>(fPrPK)) {
          JT.resize() = dynamic_cast<LinearTranslation*>(fPrPK)->getTranslationVectors();
        } 
        PJT(Index(0,2), Index(0,JT.cols()-1)) = JT;
        PJTs(Index(0,2), Index(0,JT.cols()-1)) = JT;
        PJTs(Index(0,2), Index(JT.cols(),JT.cols()+forceDir.cols()-1)) = forceDir;
      }
      if(fPJR==0) {
        Mat JR(3,0);

	if(dynamic_cast<RotationAboutFixedAxis*>(fAPK)) 
	  JR.resize() = dynamic_cast<RotationAboutFixedAxis*>(fAPK)->getAxisOfRotation();
	else if(dynamic_cast<RotationAboutAxesYZ*>(fAPK)) {
	  fPJR = new JRotationAboutAxesYZ(uSize[0]);
	  fPdJR = new JdRotationAboutAxesYZ(uSize[0]);
	}
	else if(dynamic_cast<RotationAboutAxesXY*>(fAPK)) {
	  fPJR = new JRotationAboutAxesXY(uSize[0]);
	  fPdJR = new JdRotationAboutAxesXY(uSize[0]);
	}
	else if(dynamic_cast<CardanAngles*>(fAPK)) {
	  JR.resize() << DiagMat(3,INIT,1);
	  if(cb) {
	    fT = new TCardanAngles2(qSize,uSize[0]);
	  }
	  else {
	    fT = new TCardanAngles(qSize,uSize[0]);
	  }
	}

        Mat JRR(3, uSize[0]);
        PJR(Index(0,2), Index(uSize[0]-JR.cols(),uSize[0]-1)) = JR;
        PJRs(Index(0,2), Index(uSize[1]-momentDir.cols()-JR.cols(),uSize[1]-momentDir.cols()-1)) = JR;
        PJRs(Index(0,2), Index(uSize[1]-momentDir.cols(),uSize[1]-1)) = momentDir;

        if(cb) {
          if(iKinematics == 0 && false) {
            updateM_ = &RigidBody::updateMConst;
            Mbuf = m*JTJ(PJT) + JTMJ(SThetaS,PJR);
            LLM = facLL(Mbuf);
            facLLM_ = &RigidBody::facLLMConst;
          }
          PJR0 = PJR;
        } 
        else {
        }
      }

      if(iInertia != 0)
        SThetaS = SymMat(ASF[iInertia]*SThetaS*trans(ASF[iInertia])) - m*JTJ(tilde(SrSF[iInertia]));

      for(int i=0; i<uSize[0]; i++) 
        T(i,i) = 1;
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(globalPosition)==enabled) {
          plotColumns.push_back("WxOS");
          plotColumns.push_back("WyOS");
          plotColumns.push_back("WzOS");
          plotColumns.push_back("alpha");
          plotColumns.push_back("beta");
          plotColumns.push_back("gamma");
        }
        Body::init(stage);
      }
    }
    else
      Body::init(stage);
  }

  void RigidBody::resizeJacobians(int j) {
    for(unsigned int i=0; i<frame.size(); i++) 
      frame[i]->resizeJacobians(j);

    for(unsigned int i=0; i<contour.size(); i++)
      contour[i]->resizeJacobians(j);
  }

  void RigidBody::checkForConstraints() {
    if(forceDir.cols()+momentDir.cols()) {
      Joint *joint = new Joint(string("Joint.")+name);
      ds->addInverseKineticsLink(joint);
      //ds->addLink(joint);
      if(forceDir.cols()) {
        joint->setForceDirection(forceDir);
        joint->setForceLaw(new BilateralConstraint);
        joint->setImpactForceLaw(new BilateralImpact);
      }
      if(momentDir.cols()) {
        joint->setMomentDirection(momentDir);
        joint->setMomentLaw(new BilateralConstraint);
        joint->setImpactMomentLaw(new BilateralImpact);
      }
      joint->connect(frameOfReference,frame[iKinematics]);
    }
  }

  void RigidBody::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
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
      PJT = (*fPJT)(q,t);
    if(fPJR)
      PJR = (*fPJR)(q,t);

    if(fPjT)
      PjT = (*fPjT)(t);
    if(fPjR)
      PjR = (*fPjR)(t);

    if(fAPK)
      APK = (*fAPK)(q,t);
    if(fPrPK)
      PrPK = (*fPrPK)(q,t);

    frame[iKinematics]->setOrientation(frameOfReference->getOrientation()*APK);

    if(cb) {
      PJR = trans(frameOfReference->getOrientation())*frame[iKinematics]->getOrientation()*PJR0;
    }

    WrPK = frameOfReference->getOrientation()*PrPK;
    WomPK = frameOfReference->getOrientation()*(PJR*u + PjR);
    WvPKrel = frameOfReference->getOrientation()*(PJT*u + PjT);

    frame[iKinematics]->setAngularVelocity(frameOfReference->getAngularVelocity() + WomPK);
    frame[iKinematics]->setPosition(WrPK + frameOfReference->getPosition());
    frame[iKinematics]->setVelocity(frameOfReference->getVelocity() + WvPKrel + crossProduct(frameOfReference->getAngularVelocity(),WrPK));
  }

  void RigidBody::updateJacobiansForSelectedFrame(double t) {
    if(fPdJT)
      PdJT = (*fPdJT)(T*u,q,t);
    if(fPdJR)
      PdJR = (*fPdJR)(T*u,q,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    SqrMat tWrPK = tilde(WrPK);
    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*u + PdjT) + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*u + PdjR) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(hSize[0]-uSize[0],hSize[0]-1)) = frameOfReference->getOrientation()*PJT;
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(hSize[0]-uSize[0],hSize[0]-1)) = frameOfReference->getOrientation()*PJR;
  }

  void RigidBody::updateKinematicsForRemainingFramesAndContours(double t) {
    if(iKinematics != 0) // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      frame[0]->setOrientation(frame[iKinematics]->getOrientation()*trans(ASF[iKinematics]));

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
    WThetaS = JTMJ(SThetaS,trans(frame[0]->getOrientation()));
  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours(double t) {

    if(iKinematics != 0) { // only if kinematics frame is not cog-frame, update JACOBIAN of cog
      SqrMat tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation() + tWrSK*frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation() + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[iKinematics]->getAngularVelocity(),crossProduct(frame[iKinematics]->getAngularVelocity(),-WrSF[iKinematics])));
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation());
    }

    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
        SqrMat tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrSK*frame[0]->getJacobianOfRotation());
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation());
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i])));
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
      }
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setReferenceJacobianOfRotation(frame[0]->getJacobianOfRotation());
      contour[i]->setReferenceGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
      contour[i]->setReferenceJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrSC*frame[0]->getJacobianOfRotation());
      contour[i]->setReferenceGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])));
    }
  }

  void RigidBody::updateInverseKineticsJacobiansForSelectedFrame(double t) {
    //if(forceDir.cols()+momentDir.cols()) {
      SqrMat tWrPK = tilde(WrPK);
    //  frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    //  frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

      frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
      frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();
      frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(hSize[1]-uSize[1],hSize[1]-1)) = frameOfReference->getOrientation()*PJTs;
      frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(hSize[1]-uSize[1],hSize[1]-1)) = frameOfReference->getOrientation()*PJRs;
    //} else 
      //updateJacobiansForSelectedFrame(t);
  }

  void RigidBody::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void RigidBody::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
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

  void RigidBody::updateMConst(double t) {
    M += Mbuf;
  }

  void RigidBody::updateMNotConst(double t) {
    M += m*JTJ(frame[0]->getJacobianOfTranslation()) + JTMJ(WThetaS,frame[0]->getJacobianOfRotation());
  }

  void RigidBody::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Body::initializeUsingXML(element);

    // frames
    e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
    while(e && e->ValueStr()==MBSIMNS"frame") {
      TiXmlElement *ec=e->FirstChildElement();
      Frame *f=new Frame(ec->Attribute("name"));
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
    setFrameForKinematics(getFrameByPath(e->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
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
        setOpenMBVFrameOfReference(getFrameByPath(e->FirstChildElement(MBSIMNS"frameOfReference")->Attribute("ref"))); // must be on of "Frame[X]" which allready exists
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

