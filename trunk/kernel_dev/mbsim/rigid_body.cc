/* Copyright (C) 2004-2008  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include <mbsim/rigid_body.h>
#include <mbsim/frame.h>
#include <mbsim/contour.h>
#include <mbsim/tree.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/class_factory.h>
#include <mbsim/joint.h>
#include <mbsim/constitutive_laws.h>
#ifdef HAVE_AMVIS
#include "crigidbody.h"
#include <mbsim/data_interface_base.h>
using namespace AMVis;
#endif
#include <mbsim/utils/rotarymatrices.h>
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/rigidbody.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), cb(false), m(0), SThetaS(3), WThetaS(3), iKinematics(-1), iInertia(-1), PjT(3), PjR(3), PdjT(3), PdjR(3), APK(3), PrPK(3), WrPK(3), WvPKrel(3), WomPK(3), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0) {

    APK(0,0)=1.0;
    APK(1,1)=1.0;
    APK(2,2)=1.0;

    frameOfReference = 0;

    Body::addFrame(new Frame("C"));

    SrSF.push_back(Vec(3));
    WrSF.push_back(Vec(3));
    ASF.push_back(SqrMat(3,EYE));
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

  void RigidBody::init() {
    if(iKinematics == -1)
      iKinematics = 0;
    //if(frameOfReference == 0)
    //   frameOfReference = parent->getFrame("I");

    Body::init();

    PJT.resize(3,uSize[0]);
    PJR.resize(3,uSize[0]);

    PdJT.resize(3,uSize[0]);
    PdJR.resize(3,uSize[0]);

    PJTs.resize(3,uSize[1]);
    PJRs.resize(3,uSize[1]);

    if(fPJT==0) {
      Mat JT;
      LinearTranslation* fPrPK_ = dynamic_cast<LinearTranslation*>(fPrPK);
      if(fPrPK_) {
        JT = fPrPK_->getPJT();
      } else
        JT.resize(3,0);
      Mat JTT(3, uSize[0]);
      PJT(Index(0,2), Index(0,JT.cols()-1)) = JT;
      PJTs(Index(0,2), Index(0,JT.cols()-1)) = JT;
      PJTs(Index(0,2), Index(JT.cols(),JT.cols()+forceDir.cols()-1)) = forceDir;
    }
    if(fPJR==0) {
      Mat JR;
      JR.resize(3,0);
      {
        RotationAboutFixedAxis* fAPK_ = dynamic_cast<RotationAboutFixedAxis*>(fAPK);
        if(fAPK_) 
          JR.resize() = fAPK_->getAxisOfRotation();
      }
      {
        CardanAngles* fAPK_ = dynamic_cast<CardanAngles*>(fAPK);
        if(fAPK_) {
          JR.resize() << DiagMat(3,INIT,1);
          if(cb) {
            fT = new TCardanAngles2(qSize,uSize[0]);
          } else {
            fT = new TCardanAngles(qSize,uSize[0]);
          }
        }
      }
      Mat JRR(3, uSize[0]);
      PJR(Index(0,2), Index(uSize[0]-JR.cols(),uSize[0]-1)) = JR;
      PJRs(Index(0,2), Index(uSize[1]-momentDir.cols()-JR.cols(),uSize[1]-momentDir.cols()-1)) = JR;
      PJRs(Index(0,2), Index(uSize[1]-momentDir.cols(),uSize[1]-1)) = momentDir;

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;

      if(cb) {
        // cout << "Benüzte körperfestes KOSY für Rot" << endl;
        if(iKinematics == 0 && false) {
          updateM_ = &RigidBody::updateMConst;
          Mbuf = m*JTJ(PJT) + JTMJ(SThetaS,PJR);
          LLM = facLL(Mbuf);
          facLLM_ = &RigidBody::facLLMConst;
        }
        PJR0 = PJR;
        //fPJR = new PJRTest(frame[iKinematics],frameOfReference,PJR);
      } else {
        //cout << "Benüzte Parent-KOSY für Rot" << endl;
      }
    }

    // Umrechnen des Trägheitstensor auf Schwerpunkt im Schwerpunkt-KOSY
    // Eigentlich SThetaS = ASR*RThetaR*ASR' - m*tSrSR'*tSrSR
    if(iInertia != 0)
      SThetaS = SymMat(ASF[iInertia]*SThetaS*trans(ASF[iInertia])) - m*JTJ(tilde(SrSF[iInertia]));

    for(int i=0; i<uSize[0]; i++) 
      T(i,i) = 1;

  }

  void RigidBody::initPlot() {
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

#ifdef HAVE_AMVIS
      if(bodyAMVis && getPlotFeature(amvis)==enabled)
        bodyAMVis->writeBodyFile();
#endif
      Body::initPlot();
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

#ifdef HAVE_AMVISCPPINTERFACE
      if(getPlotFeature(amvis)==enabled && amvisBody) {
        vector<double> data;
        data.push_back(t);
        Vec WrOS=frame[0]->getPosition();
        Vec cardan=AIK2Cardan(frame[0]->getOrientation());
        data.push_back(WrOS(0));
        data.push_back(WrOS(1));
        data.push_back(WrOS(2));
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        ((AMVis::RigidBody*)amvisBody)->append(data);
      }
#endif
#ifdef HAVE_AMVIS
      if(bodyAMVis && getPlotFeature(amvis)==enabled) {
        Vec WrOS=cosyAMVis->getPosition();
        Vec cardan=AIK2Cardan(cosyAMVis->getOrientation());
        bodyAMVis->setTime(t);
        bodyAMVis->setTranslation(WrOS(0),WrOS(1),WrOS(2));
        bodyAMVis->setRotation(cardan(0),cardan(1),cardan(2));
        if(bodyAMVisUserFunctionColor) {
          double color=(*bodyAMVisUserFunctionColor)(t)(0);
          if(color>1) color=1;
          if(color<0) color=0;
          bodyAMVis->setColor(color);
        }
        bodyAMVis->appendDataset(0);
      }
#endif
      Body::plot(t,dt);
    }
  }


  void RigidBody::updateMConst(double t) {
    M += Mbuf;
    //M += m*JTJ(frame[0]->getJacobianOfTranslation()) + JTMJ(WThetaS,frame[0]->getJacobianOfRotation());
  }

  void RigidBody::updateMNotConst(double t) {
    M += m*JTJ(frame[0]->getJacobianOfTranslation()) + JTMJ(WThetaS,frame[0]->getJacobianOfRotation());
  }

  void RigidBody::updateJacobiansForSelectedFrame(double t) {
    // TODO: Abfrage ob Inertiales System zur Performancesteigerung

    if(fPdJT)
      PdJT = (*fPdJT)(T*u,q,t);
    if(fPdJR)
      PdJR = (*fPdJR)(T*u,q,t);

    if(fPdjT)
      PdjT = (*fPdjT)(t);
    if(fPdjR)
      PdjR = (*fPdjR)(t);

    // Jacobi des Referenz-KOSY updaten, ausgehend vom Eltern-KOSY
    SqrMat tWrPK = tilde(WrPK);
    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJT*u + PdjT) + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + frameOfReference->getOrientation()*(PdJR*u + PdjR) + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(hSize[0]-uSize[0],hSize[0]-1)) = frameOfReference->getOrientation()*PJT;
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(hSize[0]-uSize[0],hSize[0]-1)) = frameOfReference->getOrientation()*PJR;
  }

  void RigidBody::updateSecondJacobiansForSelectedFrame(double t) {
    //  if(uSize == usSize)
    //    updateJacobiansForSelectedFrame(t);
    //  else 
    //  {
    //  // TODO: Abfrage ob Inertiales System zur Performancesteigerung
    // 
    //   if(fPdJT)
    //     PdJT = (*fPdJT)(T*u,q,t);
    //   if(fPdJR)
    //     PdJR = (*fPdJR)(T*u,q,t);

    //   if(fPdjT)
    //     PdjT = (*fPdjT)(t);
    //   if(fPdjR)
    //     PdjR = (*fPdjR)(t);

    // Jacobi des Referenz-KOSY updaten, ausgehend vom Eltern-KOSY
    SqrMat tWrPK = tilde(WrPK);
    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), WomPK));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();
    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(hSize[1]-uSize[1],hSize[1]-1)) = frameOfReference->getOrientation()*PJTs;
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(hSize[1]-uSize[1],hSize[1]-1)) = frameOfReference->getOrientation()*PJRs;
    //cout << name << endl;
    //cout << usSize << endl;
    //cout << hsSize << endl;
    //cout << PJTs << endl;
    //cout << PJRs << endl;
    //  }
  }

  void RigidBody::updateJacobiansForRemainingFramesAndContours(double t) {

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Jacobi des Schwerpunkt-KOSY updaten
    if(iKinematics != 0) {
      SqrMat tWrSK = tilde(WrSF[iKinematics]);
      frame[0]->setJacobianOfTranslation(frame[iKinematics]->getJacobianOfTranslation() + tWrSK*frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setJacobianOfRotation(frame[iKinematics]->getJacobianOfRotation());
      frame[0]->setGyroscopicAccelerationOfTranslation(frame[iKinematics]->getGyroscopicAccelerationOfTranslation() + tWrSK*frame[iKinematics]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[iKinematics]->getAngularVelocity(),crossProduct(frame[iKinematics]->getAngularVelocity(),-WrSF[iKinematics])));
      frame[0]->setGyroscopicAccelerationOfRotation(frame[iKinematics]->getGyroscopicAccelerationOfRotation());
    }

    // Jacobi der anderen KOSY (außer Schwerpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
        SqrMat tWrSK = tilde(WrSF[i]);
        frame[i]->setJacobianOfTranslation(frame[0]->getJacobianOfTranslation() - tWrSK*frame[0]->getJacobianOfRotation());
        frame[i]->setJacobianOfRotation(frame[0]->getJacobianOfRotation());
        frame[i]->setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSK*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSF[i])));
        frame[i]->setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation());
      }
    }

    // Jacobi der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setWJR(frame[0]->getJacobianOfRotation());
      contour[i]->setWjR(frame[0]->getGyroscopicAccelerationOfRotation());
      contour[i]->setWJP(frame[0]->getJacobianOfTranslation() - tWrSC*frame[0]->getJacobianOfRotation());
      contour[i]->setWjP(frame[0]->getGyroscopicAccelerationOfTranslation() - tWrSC*frame[0]->getGyroscopicAccelerationOfRotation() + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrSC[i])));
    }
  }

  void RigidBody::updateh(double t) {

    // Trägheitstensor auf Welt-System umrechnen
    //WThetaS =
    //SymMat(frame[0]->getOrientation()*SThetaS*trans(frame[0]->getOrientation()));
    WThetaS = JTMJ(SThetaS,trans(frame[0]->getOrientation()));

    Vec WF = m*ds->getAccelerationOfGravity() - m*frame[0]->getGyroscopicAccelerationOfTranslation();
    Vec WM = crossProduct(WThetaS*frame[0]->getAngularVelocity(),frame[0]->getAngularVelocity()) - WThetaS*frame[0]->getGyroscopicAccelerationOfRotation();

    h += trans(frame[0]->getJacobianOfTranslation())*WF +  trans(frame[0]->getJacobianOfRotation())*WM;

  }

  void RigidBody::resizeJacobians(int j) {

    for(unsigned int i=0; i<frame.size(); i++) 
      frame[i]->resizeJacobians(j);

    for(unsigned int i=0; i<contour.size(); i++)
      contour[i]->resizeJacobians(j);
  }

  void RigidBody::checkForConstraints() {
    if(uSize[1] > uSize[0]) {
      Joint *joint = new Joint(string("Joint.")+name);
      cout << joint->getName() << endl;
      parent->addLink(joint);
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


  //  void RigidBody::updateVRef(const Mat& VParent) {
  //    Frame* cos[2];
  //    cos[0] = frameOfReference;
  //    cos[1] = frame[0];
  //    for(unsigned i=0; i<22; i++) {
  //      int hInd = cos[i]->getParent()->gethInd(parent);
  //      Index J = Index(laInd,laInd+laSize-1);
  //      Index I = Index(hInd,hInd+cos[i]->gethSize()-1);
  //      V[i]>>VParent(I,J);
  //    }
  //  } 
  //
  //  void RigidBody::updateWRef(const Mat& WParent) {
  //    Frame* cos[2];
  //    cos[0] = frameOfReference;
  //    cos[1] = frame[0];
  //    for(unsigned i=0; i<22; i++) {
  //      int hInd = cos[i]->getParent()->gethInd(parent);
  //      Index J = Index(laInd,laInd+laSize-1);
  //      Index I = Index(hInd,hInd+cos[i]->gethSize()-1);
  //      W[i]>>WParent(I,J);
  //    }
  //  } 
  //
  //  void RigidBody::updatewbRef(const Vec& wbParent) {
  //    wb.resize() >> wbParent(laInd,laInd+laSize-1);
  //  }
  //
  //  void RigidBody::updateW(double t) {
  //    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
  //    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
  //    fF[1] = -fF[0];
  //    fM[1] = -fM[0];
  //
  //    W[0] += trans(C.getJacobianOfTranslation())*fF[0] + trans(C.getJacobianOfRotation())*fM[0];
  //    W[1] += trans(frame[1]->getJacobianOfTranslation())*fF[1] + trans(frame[1]->getJacobianOfRotation())*fM[1];
  //  }
  //
  //  void RigidBody::updatewb(double t) {
  //
  //    Mat WJT = frame[0]->getOrientation()*JT;
  //    Vec sdT = trans(WJT)*(WvP0P1);
  //
  //    wb(0,Wf.cols()-1) += trans(Wf)*(frame[1]->getGyroscopicAccelerationOfTranslation() - C.getGyroscopicAccelerationOfTranslation() - crossProduct(C.getAngularVelocity(),WvP0P1+WJT*sdT));
  //    wb(Wf.cols(),Wm.cols()+Wf.cols()-1) += trans(Wm)*(frame[1]->getGyroscopicAccelerationOfRotation() - C.getGyroscopicAccelerationOfRotation() - crossProduct(C.getAngularVelocity(),WomP0P1));
  //  }
  //


  void RigidBody::updateKinematicsForSelectedFrame(double t) {

    // TODO: Abfrage ob Inertiales System zur Performancesteigerung

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

    // Kinematik des Referenz-KOSY updaten
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

  void RigidBody::updateKinematicsForRemainingFramesAndContours(double t) {

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Drehmatrix des Schwerpunkt-KOSY updaten
    if(iKinematics != 0)
      frame[0]->setOrientation(frame[iKinematics]->getOrientation()*trans(ASF[iKinematics]));

    // Ortsvektoren vom Schwerpunkt-KOSY zu anderen KOSY im Welt-KOSY 
    for(unsigned int i=1; i<frame.size(); i++) {
      WrSF[i] = frame[0]->getOrientation()*SrSF[i];
    }

    // Ortsvektoren vom Schwerpunkt-KOSY zu Konturen im Welt-KOSY 
    for(unsigned int i=0; i<contour.size(); i++) {
      WrSC[i] = frame[0]->getOrientation()*SrSC[i];
    }

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Kinematik des Schwerpunkt-KOSY updaten
    if(iKinematics != 0) {
      frame[0]->setPosition(frame[iKinematics]->getPosition() - WrSF[iKinematics]);
      frame[0]->setVelocity(frame[iKinematics]->getVelocity() - crossProduct(frame[iKinematics]->getAngularVelocity(), WrSF[iKinematics]));
      frame[0]->setAngularVelocity(frame[iKinematics]->getAngularVelocity());
    }

    // Kinematik der anderen KOSY (außer Scherpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<frame.size(); i++) {
      if(i!=unsigned(iKinematics)) {
        frame[i]->setPosition(frame[0]->getPosition() + WrSF[i]);
        frame[i]->setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(), WrSF[i]));
        frame[i]->setAngularVelocity(frame[0]->getAngularVelocity());
        frame[i]->setOrientation(frame[0]->getOrientation()*ASF[i]);
      }
    }
    // Kinematik der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setAWC(frame[0]->getOrientation()*ASC[i]);
      contour[i]->setWomegaC(frame[0]->getAngularVelocity());
      contour[i]->setWrOP(frame[0]->getPosition() + WrSC[i]);
      contour[i]->setWvP(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(), WrSC[i]));
    }
  }

  double RigidBody::computeKineticEnergy() {
    //return 0.5 * (m*trans(KvK)*(KvK + 2*crossProduct(KomegaK,KrKS)) + trans(KomegaK)*I*KomegaK);
    return 0;
  }

  double RigidBody::computeKineticEnergyBranch() {
    // double Ttemp = computeKineticEnergy();
    // for(unsigned int i=0; i<successor.size(); i++)
    //   Ttemp += successor[i]->computeKineticEnergyBranch();
    // return Ttemp;
    return -1;
  }

  double RigidBody::computePotentialEnergyBranch() {
    //  double Vbranch = this->computePotentialEnergy();
    //  for(unsigned int i=0; i<successor.size(); i++)
    //    Vbranch += successor[i]->computePotentialEnergy();
    //  return Vbranch;
    return -1;
  }

  void RigidBody::addFrame(Frame *cosy, const Vec &RrRK, const SqrMat &ARK, const Frame* refFrame) {
    Body::addFrame(cosy);
    int i = 0;
    if(refFrame)
      i = frameIndex(refFrame);

    SrSF.push_back(SrSF[i] + ASF[i]*RrRK);
    WrSF.push_back(Vec(3));
    ASF.push_back(ASF[i]*ARK);
  }

  void RigidBody::addFrame(const string &str, const Vec &SrSF, const SqrMat &ASF, const Frame* refFrame) {
    addFrame(new Frame(str),SrSF,ASF,refFrame);
  }

  void RigidBody::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const Frame* refFrame) {
    Body::addContour(contour);

    int i = 0;
    if(refFrame)
      i = frameIndex(refFrame);

    SrSC.push_back(SrSF[i] + ASF[i]*RrRC);
    WrSC.push_back(Vec(3));
    ASC.push_back(ASF[i]*ARC);

    // HitSphere anpassen !!!
    //contour->adjustParentHitSphere(SrSC[SrSC.size()-1]);
  }

}

