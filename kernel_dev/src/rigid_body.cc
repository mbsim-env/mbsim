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
#include "rigid_body.h"
#include "coordinate_system.h"
#include "contour.h"
#include "link.h"
#include "tree.h"
#include "multi_body_system.h"
#ifdef HAVE_AMVIS
#include "crigidbody.h"
#include "data_interface_base.h"
using namespace AMVis;
#endif


namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), cb(false), m(0), SThetaS(3), WThetaS(3),iRef(-1), i4I(-1), PjT(3), PjR(3), PdjT(3), PdjR(3), APK(3), PrPK(3), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0) {

    APK(0,0)=1.0;
    APK(1,1)=1.0;
    APK(2,2)=1.0;

    portParent = 0;

    Object::addCoordinateSystem(new CoordinateSystem("COG"));

    SrSK.push_back(Vec(3));
    WrSK.push_back(Vec(3));
    ASK.push_back(SqrMat(3,EYE));
  }

  void RigidBody::calcSize() {

    Body::calcSize();
    qSize = 0;
    uSize = 0;
    if(fPrPK)
      qSize += fPrPK->getqSize();
    if(fAPK)
      qSize += fAPK->getqSize();
    if(fPJT==0) {
      LinearTranslation* fPrPK_ = dynamic_cast<LinearTranslation*>(fPrPK);
      if(fPrPK_) 
	uSize += fPrPK->getqSize();
    } else
      uSize += fPJT->getuSize();
    if(fPJR==0) {
      //TODO  Euler-Parameter
      //RotationAxis* fAPK_ = dynamic_cast<RotationAxis*>(fAPK);
      if(fAPK)
	uSize += fAPK->getqSize();
      //CardanAngles* fAPK_ = dynamic_cast<CardanAngles*>(fAPK);
      //if(fAPK_)
	//uSize += fAPK->getqSize();
    } else
      uSize += fPJR->getuSize();

  }

  void RigidBody::init() {
    Object::init();

    PJT.resize(3,uSize);
    PJR.resize(3,uSize);

    PdJT.resize(3,uSize);
    PdJR.resize(3,uSize);
    if(fPJT==0) {
      Mat JT;
      LinearTranslation* fPrPK_ = dynamic_cast<LinearTranslation*>(fPrPK);
      if(fPrPK_) {
	JT = fPrPK_->getPJT();
      } else
	JT.resize(3,0);
      Mat JTT(3, uSize);
      PJT(Index(0,2), Index(0,JT.cols()-1)) = JT;
    }
    if(fPJR==0) {
      Mat JR;
      JR.resize(3,0);
      {
	RotationAxis* fAPK_ = dynamic_cast<RotationAxis*>(fAPK);
	if(fAPK_) 
	  JR.resize() = fAPK_->getAxis();
      }
      {
	CardanAngles* fAPK_ = dynamic_cast<CardanAngles*>(fAPK);
	if(fAPK_) {
	  JR.resize() << DiagMat(3,INIT,1);
	  if(cb) {
	    fT = new TCardanAngles2(qSize,uSize);
	  } else {
	    fT = new TCardanAngles(qSize,uSize);
	  }
	}
      }
      Mat JRR(3, uSize);
      PJR(Index(0,2), Index(uSize-JR.cols(),uSize-1)) = JR;

      updateM_ = &RigidBody::updateMNotConst;
      facLLM_ = &RigidBody::facLLMNotConst;

      if(cb) {
	 // cout << "Benüzte körperfestes KOSY für Rot" << endl;
	  if(iRef == 0 && false) {
	    updateM_ = &RigidBody::updateMConst;
	    Mbuf = m*JTJ(PJT) + JTMJ(SThetaS,PJR);
	    LLM = facLL(Mbuf);
	    facLLM_ = &RigidBody::facLLMConst;
	  }
	  PJR0 = PJR;
	  //fPJR = new PJRTest(port[iRef],portParent,PJR);
      } else {
	//cout << "Benüzte Parent-KOSY für Rot" << endl;
      }
    }

    // Umrechnen des Trägheitstensor auf Schwerpunkt im Schwerpunkt-KOSY
    // Eigentlich SThetaS = ASR*RThetaR*ASR' - m*tSrSR'*tSrSR
    if(i4I != 0)
      SThetaS = SymMat(ASK[i4I]*SThetaS*trans(ASK[i4I])) - m*JTJ(tilde(SrSK[i4I]));

    for(int i=0; i<uSize; i++) 
      T(i,i) = 1;

    for(unsigned int i=0; i<port.size(); i++) {
      port[i]->getWJP().resize(3,hSize);
      port[i]->getWJR().resize(3,hSize);
    }
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->getWJP().resize(3,hSize);
      contour[i]->getWJR().resize(3,hSize);
    }
      
  }

  void RigidBody::initPlotFiles() {

      Body::initPlotFiles();

#ifdef HAVE_AMVIS
    if(bodyAMVis)
      bodyAMVis->writeBodyFile();
#endif

    if(plotLevel>0) {
      plotfile <<"# "<< plotNr++ << ": WxOS" << endl;
      plotfile <<"# "<< plotNr++ << ": WyOS" << endl;
      plotfile <<"# "<< plotNr++ << ": WzOS" << endl;
      plotfile <<"# "<< plotNr++ << ": alpha" << endl;
      plotfile <<"# "<< plotNr++ << ": beta" << endl;
      plotfile <<"# "<< plotNr++ << ": gamma" <<endl;

      if(plotLevel>2) {
	plotfile <<"# "<< plotNr++ << ": T" << endl;
	plotfile <<"# "<< plotNr++ << ": V" << endl;
	plotfile <<"# "<< plotNr++ << ": E" << endl;
      }

    }

  }

  void RigidBody::updateMConst(double t) {
      M += Mbuf;
      //M += m*JTJ(port[0]->getWJP()) + JTMJ(WThetaS,port[0]->getWJR());
  }

  void RigidBody::updateMNotConst(double t) {
      M += m*JTJ(port[0]->getWJP()) + JTMJ(WThetaS,port[0]->getWJR());
  }

  void RigidBody::updateJacobians(double t) {
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
    port[iRef]->setWjP(portParent->getWjP() - tWrPK*portParent->getWjR() + portParent->getAWP()*(PdJT*u + PdjT) + crossProduct(portParent->getWomegaP(), 2*WvPKrel+crossProduct(portParent->getWomegaP(),WrPK)));
    port[iRef]->setWjR(portParent->getWjR() + portParent->getAWP()*(PdJR*u + PdjR) + crossProduct(portParent->getWomegaP(), WomPK));

    port[iRef]->getWJP()(Index(0,2),Index(0,portParent->getWJP().cols()-1)) = portParent->getWJP() - tWrPK*portParent->getWJR();
    port[iRef]->getWJR()(Index(0,2),Index(0,portParent->getWJR().cols()-1)) = portParent->getWJR();
    port[iRef]->getWJP()(Index(0,2),Index(hSize-uSize,hSize-1)) = portParent->getAWP()*PJT;
    port[iRef]->getWJR()(Index(0,2),Index(hSize-uSize,hSize-1)) = portParent->getAWP()*PJR;

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Jacobi des Schwerpunkt-KOSY updaten
    if(iRef != 0) {
      SqrMat tWrSK = tilde(WrSK[iRef]);
      port[0]->setWJP(port[iRef]->getWJP() + tWrSK*port[iRef]->getWJR());
      port[0]->setWJR(port[iRef]->getWJR());
      port[0]->setWjP(port[iRef]->getWjP() + tWrSK*port[iRef]->getWjR() + crossProduct(port[iRef]->getWomegaP(),crossProduct(port[iRef]->getWomegaP(),-WrSK[iRef])));
      port[0]->setWjR(port[iRef]->getWjR());
    }

    // Jacobi der anderen KOSY (außer Schwerpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<port.size(); i++) {
      if(i!=unsigned(iRef)) {
	SqrMat tWrSK = tilde(WrSK[i]);
	port[i]->setWJP(port[0]->getWJP() - tWrSK*port[0]->getWJR());
	port[i]->setWJR(port[0]->getWJR());
	port[i]->setWjP(port[0]->getWjP() - tWrSK*port[0]->getWjR() + crossProduct(port[0]->getWomegaP(),crossProduct(port[0]->getWomegaP(),WrSK[i])));
	port[i]->setWjR(port[0]->getWjR());
      }
    }

     // Jacobi der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setWJP(port[0]->getWJP() - tWrSC*port[0]->getWJR());
      contour[i]->setWJR(port[0]->getWJR());
      contour[i]->setWjP(port[0]->getWjP() - tWrSC*port[0]->getWjR() + crossProduct(port[0]->getWomegaP(),crossProduct(port[0]->getWomegaP(),WrSC[i])));
      contour[i]->setWjR(port[0]->getWjR());
    }
  }

  void RigidBody::updateh(double t) {
    updateJacobians(t);

    // Trägheitstensor auf Welt-System umrechnen
    //WThetaS = SymMat(port[0]->getAWP()*SThetaS*trans(port[0]->getAWP()));
    WThetaS = JTMJ(SThetaS,trans(port[0]->getAWP()));

    Vec WF = m*parent->getAccelerationOfGravity() - m*port[0]->getWjP();
    Vec WM = crossProduct(WThetaS*port[0]->getWomegaP(),port[0]->getWomegaP()) - WThetaS*port[0]->getWjR();

    h += trans(port[0]->getWJP())*WF +  trans(port[0]->getWJR())*WM;

  }

 void RigidBody::plot(double t, double dt) {
   Body::plot(t,dt);
   //Vec WrOS = computeWrOS();
   SqrMat AWK = port[0]->getAWP();
   Vec WrOS = port[0]->getWrOP();
#ifdef HAVE_AMVIS
    if(plotLevel>0 || bodyAMVis)
#else
      if(plotLevel>0)
#endif
      {
	double alpha;
	double beta=asin(AWK(0,2));
	double gamma;
	double nenner=cos(beta);
	if (nenner>1e-10) {
	  alpha=atan2(-AWK(1,2),AWK(2,2));
	  gamma=atan2(-AWK(0,1),AWK(0,0));
	} else {
	  alpha=0;
	  gamma=atan2(AWK(1,0),AWK(1,1));
	}

	if(plotLevel>0) {
	  plotfile<<" "<<WrOS(0)<<" "<<WrOS(1)<<" "<<WrOS(2);
	  plotfile<<" "<<alpha<<" "<<beta<<" "<<gamma; 

	  if(plotLevel>2) {
	    double Ttemp = computeKineticEnergy();
	    double Vtemp = computePotentialEnergy();
	    plotfile <<" "<<  Ttemp;
	    plotfile <<" "<<  Vtemp;
	    plotfile <<" "<< Ttemp+Vtemp;
	  }
	}

#ifdef HAVE_AMVIS
	if(bodyAMVis) {
	  bodyAMVis->setTime(t);
	  bodyAMVis->setTranslation(WrOS(0),WrOS(1),WrOS(2));
	  bodyAMVis->setRotation(alpha,beta,gamma);
	  if (bodyAMVisUserFunctionColor) {
	    double color = (*bodyAMVisUserFunctionColor)(t)(0);
	    if (color>1) color =1;
	    if (color<0) color =0;
	    bodyAMVis->setColor(color);
	  }
	  bodyAMVis->appendDataset(0);
	}
#endif
      }
  }

  void RigidBody::updateKinematics(double t) {

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
    port[iRef]->setAWP(portParent->getAWP()*APK);

    if(cb) {
      PJR = trans(portParent->getAWP())*port[iRef]->getAWP()*PJR0;
    }

    WrPK = portParent->getAWP()*PrPK;
    WomPK = portParent->getAWP()*(PJR*u + PjR);
    WvPKrel = portParent->getAWP()*(PJT*u + PjT);
    port[iRef]->setWomegaP(portParent->getWomegaP() + WomPK);
    port[iRef]->setWrOP(WrPK + portParent->getWrOP());
    port[iRef]->setWvP(portParent->getWvP() + WvPKrel + crossProduct(portParent->getWomegaP(),WrPK));

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Drehmatrix des Schwerpunkt-KOSY updaten
    if(iRef != 0)
      port[0]->setAWP(port[iRef]->getAWP()*trans(ASK[iRef]));

    // Ortsvektoren vom Schwerpunkt-KOSY zu anderen KOSY im Welt-KOSY 
    for(unsigned int i=1; i<port.size(); i++) {
      WrSK[i] = port[0]->getAWP()*SrSK[i];
    }

    // Ortsvektoren vom Schwerpunkt-KOSY zu Konturen im Welt-KOSY 
    for(unsigned int i=0; i<contour.size(); i++) {
      WrSC[i] = port[0]->getAWP()*SrSC[i];
    }

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Kinematik des Schwerpunkt-KOSY updaten
    if(iRef != 0) {
      port[0]->setWrOP(port[iRef]->getWrOP() - WrSK[iRef]);
      port[0]->setWvP(port[iRef]->getWvP() - crossProduct(port[iRef]->getWomegaP(), WrSK[iRef]));
      port[0]->setWomegaP(port[iRef]->getWomegaP());
    }

    // Kinematik der anderen KOSY (außer Scherpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<port.size(); i++) {
      if(i!=unsigned(iRef)) {
	port[i]->setWrOP(port[0]->getWrOP() + WrSK[i]);
	port[i]->setWvP(port[0]->getWvP() + crossProduct(port[0]->getWomegaP(), WrSK[i]));
	port[i]->setWomegaP(port[0]->getWomegaP());
	port[i]->setAWP(port[0]->getAWP()*ASK[i]);
      }
    }
    // Kinematik der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setWrOP(port[0]->getWrOP() + WrSC[i]);
      contour[i]->setWvP(port[0]->getWvP() + crossProduct(port[0]->getWomegaP(), WrSC[i]));
      contour[i]->setWomegaC(port[0]->getWomegaP());
      contour[i]->setAWC(port[0]->getAWP()*ASC[i]);
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

  void RigidBody::addCoordinateSystem(CoordinateSystem *cosy, const Vec &RrRK, const SqrMat &ARK, const CoordinateSystem* refCoordinateSystem) {
    Object::addCoordinateSystem(cosy);
    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    SrSK.push_back(SrSK[i] + ASK[i]*RrRK);
    WrSK.push_back(Vec(3));
    ASK.push_back(ASK[i]*ARK);
  }

  void RigidBody::addCoordinateSystem(const string &str, const Vec &SrSK, const SqrMat &ASK, const CoordinateSystem* refCoordinateSystem) {
    addCoordinateSystem(new CoordinateSystem(str),SrSK,ASK,refCoordinateSystem);
  }

  void RigidBody::addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem) {
    Object::addContour(contour);

    int i = 0;
    if(refCoordinateSystem)
      i = portIndex(refCoordinateSystem);

    SrSC.push_back(SrSK[i] + ASK[i]*RrRC);
    WrSC.push_back(Vec(3));
    ASC.push_back(ASK[i]*ARC);

    // HitSphere anpassen !!!
    contour->adjustParentHitSphere(SrSC[SrSC.size()-1]);
  }
}
