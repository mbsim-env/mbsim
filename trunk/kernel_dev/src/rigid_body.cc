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
#include "tree.h"
#include "multi_body_system.h"
#include "class_factory.h"
#ifdef HAVE_AMVIS
#include "crigidbody.h"
#include "data_interface_base.h"
using namespace AMVis;
#endif


namespace MBSim {

  RigidBody::RigidBody(const string &name) : Body(name), cb(false), m(0), SThetaS(3), WThetaS(3), iRef(-1), i4I(-1), PjT(3), PjR(3), PdjT(3), PdjR(3), APK(3), PrPK(3), fT(0), fPrPK(0), fAPK(0), fPJT(0), fPJR(0), fPdJT(0), fPdJR(0), fPjT(0), fPjR(0), fPdjT(0), fPdjR(0) {

    APK(0,0)=1.0;
    APK(1,1)=1.0;
    APK(2,2)=1.0;

    portParent = 0;

    Object::addCoordinateSystem(new CoordinateSystem("C"));

    SrSK.push_back(Vec(3));
    WrSK.push_back(Vec(3));
    ASK.push_back(SqrMat(3,EYE));
  }

  void RigidBody::calcqSize() {

    Body::calcqSize();
    qSize = 0;
    if(fPrPK)
      qSize += fPrPK->getqSize();
    if(fAPK)
      qSize += fAPK->getqSize();
  }

  void RigidBody::calcuSize() {

    Body::calcuSize();
    uSize = 0;
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
    if(iRef == -1)
      iRef = 0;
    //if(portParent == 0)
   //   portParent = parent->getCoordinateSystem("I");
    
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
	RotationAboutFixedAxis* fAPK_ = dynamic_cast<RotationAboutFixedAxis*>(fAPK);
	if(fAPK_) 
	  JR.resize() = fAPK_->getAxisOfRotation();
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

  }

  void RigidBody::initPlotFiles() {

      Body::initPlotFiles();

#ifdef HAVE_AMVIS
    if(bodyAMVis)
      bodyAMVis->writeBodyFile();
#endif

    if(plotLevel>0) {
    //  plotfile <<"# "<< plotNr++ << ": WxOS" << endl;
    //  plotfile <<"# "<< plotNr++ << ": WyOS" << endl;
    //  plotfile <<"# "<< plotNr++ << ": WzOS" << endl;
    //  plotfile <<"# "<< plotNr++ << ": alpha" << endl;
    //  plotfile <<"# "<< plotNr++ << ": beta" << endl;
    //  plotfile <<"# "<< plotNr++ << ": gamma" <<endl;

    //  if(plotLevel>2) {
    //    plotfile <<"# "<< plotNr++ << ": T" << endl;
    //    plotfile <<"# "<< plotNr++ << ": V" << endl;
    //    plotfile <<"# "<< plotNr++ << ": E" << endl;
    //  }

    }

  }

  void RigidBody::updateMConst(double t) {
    M += Mbuf;
      //M += m*JTJ(port[0]->getJacobianOfTranslation()) + JTMJ(WThetaS,port[0]->getJacobianOfRotation());
  }

  void RigidBody::updateMNotConst(double t) {
    M += m*JTJ(port[0]->getJacobianOfTranslation()) + JTMJ(WThetaS,port[0]->getJacobianOfRotation());
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
    port[iRef]->setGyroscopicAccelerationOfTranslation(portParent->getGyroscopicAccelerationOfTranslation() - tWrPK*portParent->getGyroscopicAccelerationOfRotation() + portParent->getOrientation()*(PdJT*u + PdjT) + crossProduct(portParent->getAngularVelocity(), 2*WvPKrel+crossProduct(portParent->getAngularVelocity(),WrPK)));
    port[iRef]->setGyroscopicAccelerationOfRotation(portParent->getGyroscopicAccelerationOfRotation() + portParent->getOrientation()*(PdJR*u + PdjR) + crossProduct(portParent->getAngularVelocity(), WomPK));

    port[iRef]->getJacobianOfTranslation()(Index(0,2),Index(0,portParent->getJacobianOfTranslation().cols()-1)) = portParent->getJacobianOfTranslation() - tWrPK*portParent->getJacobianOfRotation();
    port[iRef]->getJacobianOfRotation()(Index(0,2),Index(0,portParent->getJacobianOfRotation().cols()-1)) = portParent->getJacobianOfRotation();
    port[iRef]->getJacobianOfTranslation()(Index(0,2),Index(hSize-uSize,hSize-1)) = portParent->getOrientation()*PJT;
    port[iRef]->getJacobianOfRotation()(Index(0,2),Index(hSize-uSize,hSize-1)) = portParent->getOrientation()*PJR;

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Jacobi des Schwerpunkt-KOSY updaten
    if(iRef != 0) {
      SqrMat tWrSK = tilde(WrSK[iRef]);
      port[0]->setJacobianOfTranslation(port[iRef]->getJacobianOfTranslation() + tWrSK*port[iRef]->getJacobianOfRotation());
      port[0]->setJacobianOfRotation(port[iRef]->getJacobianOfRotation());
      port[0]->setGyroscopicAccelerationOfTranslation(port[iRef]->getGyroscopicAccelerationOfTranslation() + tWrSK*port[iRef]->getGyroscopicAccelerationOfRotation() + crossProduct(port[iRef]->getAngularVelocity(),crossProduct(port[iRef]->getAngularVelocity(),-WrSK[iRef])));
      port[0]->setGyroscopicAccelerationOfRotation(port[iRef]->getGyroscopicAccelerationOfRotation());
    }

    // Jacobi der anderen KOSY (außer Schwerpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<port.size(); i++) {
      if(i!=unsigned(iRef)) {
	SqrMat tWrSK = tilde(WrSK[i]);
	port[i]->setJacobianOfTranslation(port[0]->getJacobianOfTranslation() - tWrSK*port[0]->getJacobianOfRotation());
	port[i]->setJacobianOfRotation(port[0]->getJacobianOfRotation());
	port[i]->setGyroscopicAccelerationOfTranslation(port[0]->getGyroscopicAccelerationOfTranslation() - tWrSK*port[0]->getGyroscopicAccelerationOfRotation() + crossProduct(port[0]->getAngularVelocity(),crossProduct(port[0]->getAngularVelocity(),WrSK[i])));
	port[i]->setGyroscopicAccelerationOfRotation(port[0]->getGyroscopicAccelerationOfRotation());
      }
    }

     // Jacobi der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat tWrSC = tilde(WrSC[i]);
      contour[i]->setWJP(port[0]->getJacobianOfTranslation() - tWrSC*port[0]->getJacobianOfRotation());
      contour[i]->setWJR(port[0]->getJacobianOfRotation());
      contour[i]->setWjP(port[0]->getGyroscopicAccelerationOfTranslation() - tWrSC*port[0]->getGyroscopicAccelerationOfRotation() + crossProduct(port[0]->getAngularVelocity(),crossProduct(port[0]->getAngularVelocity(),WrSC[i])));
      contour[i]->setWjR(port[0]->getGyroscopicAccelerationOfRotation());
    }
  }

  void RigidBody::updateh(double t) {

    // Trägheitstensor auf Welt-System umrechnen
    //WThetaS =
    //SymMat(port[0]->getOrientation()*SThetaS*trans(port[0]->getOrientation()));
    WThetaS = JTMJ(SThetaS,trans(port[0]->getOrientation()));

    Vec WF = m*mbs->getAccelerationOfGravity() - m*port[0]->getGyroscopicAccelerationOfTranslation();
    Vec WM = crossProduct(WThetaS*port[0]->getAngularVelocity(),port[0]->getAngularVelocity()) - WThetaS*port[0]->getGyroscopicAccelerationOfRotation();

    h += trans(port[0]->getJacobianOfTranslation())*WF +  trans(port[0]->getJacobianOfRotation())*WM;

  }

 void RigidBody::plot(double t, double dt) {
   Body::plot(t,dt);
#ifdef HAVE_AMVIS
    if(plotLevel>0 || bodyAMVis)
#else
      if(plotLevel>0)
#endif
      {
	if(plotLevel>0) {
	 // plotfile<<" "<<WrOS(0)<<" "<<WrOS(1)<<" "<<WrOS(2);
	 // plotfile<<" "<<alpha<<" "<<beta<<" "<<gamma; 

	 // if(plotLevel>2) {
	 //   double Ttemp = computeKineticEnergy();
	 //   double Vtemp = computePotentialEnergy();
	 //   plotfile <<" "<<  Ttemp;
	 //   plotfile <<" "<<  Vtemp;
	 //   plotfile <<" "<< Ttemp+Vtemp;
	 // }
	}

#ifdef HAVE_AMVIS
	if(bodyAMVis) {
	  SqrMat AWK = cosyAMVis->getOrientation();
	  Vec WrOS = cosyAMVis->getPosition();
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
    port[iRef]->setOrientation(portParent->getOrientation()*APK);

    if(cb) {
      PJR = trans(portParent->getOrientation())*port[iRef]->getOrientation()*PJR0;
    }

    WrPK = portParent->getOrientation()*PrPK;
    WomPK = portParent->getOrientation()*(PJR*u + PjR);
    WvPKrel = portParent->getOrientation()*(PJT*u + PjT);
    port[iRef]->setAngularVelocity(portParent->getAngularVelocity() + WomPK);
    port[iRef]->setPosition(WrPK + portParent->getPosition());
    port[iRef]->setVelocity(portParent->getVelocity() + WvPKrel + crossProduct(portParent->getAngularVelocity(),WrPK));

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Drehmatrix des Schwerpunkt-KOSY updaten
    if(iRef != 0)
      port[0]->setOrientation(port[iRef]->getOrientation()*trans(ASK[iRef]));

    // Ortsvektoren vom Schwerpunkt-KOSY zu anderen KOSY im Welt-KOSY 
    for(unsigned int i=1; i<port.size(); i++) {
      WrSK[i] = port[0]->getOrientation()*SrSK[i];
    }

    // Ortsvektoren vom Schwerpunkt-KOSY zu Konturen im Welt-KOSY 
    for(unsigned int i=0; i<contour.size(); i++) {
      WrSC[i] = port[0]->getOrientation()*SrSC[i];
    }

    // Nur wenn Referenz-KOSY nicht Schwerpunkt-KOSY, Kinematik des Schwerpunkt-KOSY updaten
    if(iRef != 0) {
      port[0]->setPosition(port[iRef]->getPosition() - WrSK[iRef]);
      port[0]->setVelocity(port[iRef]->getVelocity() - crossProduct(port[iRef]->getAngularVelocity(), WrSK[iRef]));
      port[0]->setAngularVelocity(port[iRef]->getAngularVelocity());
    }

    // Kinematik der anderen KOSY (außer Scherpunkt- und Referenz-) updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=1; i<port.size(); i++) {
      if(i!=unsigned(iRef)) {
	port[i]->setPosition(port[0]->getPosition() + WrSK[i]);
	port[i]->setVelocity(port[0]->getVelocity() + crossProduct(port[0]->getAngularVelocity(), WrSK[i]));
	port[i]->setAngularVelocity(port[0]->getAngularVelocity());
	port[i]->setOrientation(port[0]->getOrientation()*ASK[i]);
      }
    }
    // Kinematik der Konturen updaten, ausgehend vom Schwerpuntk-KOSY
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->setWrOP(port[0]->getPosition() + WrSC[i]);
      contour[i]->setWvP(port[0]->getVelocity() + crossProduct(port[0]->getAngularVelocity(), WrSC[i]));
      contour[i]->setWomegaC(port[0]->getAngularVelocity());
      contour[i]->setAWC(port[0]->getOrientation()*ASC[i]);
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
    //contour->adjustParentHitSphere(SrSC[SrSC.size()-1]);
  }

  void RigidBody::save(const string &path, ofstream& outputfile) {
    Body::save(path,outputfile);
    // all CoordinateSystem of Object
    
    outputfile << "# Mass: " << endl;
    outputfile << m << endl << endl;

    outputfile << "# Inertia tensor: " << endl;
    outputfile << SThetaS << endl << endl;

    for(unsigned int i=1; i<port.size(); i++) {
      outputfile << "# Translation of coordinate system " << port[i]->getName() <<":" << endl;
      outputfile << SrSK[i] << endl << endl;
      outputfile << "# Rotation of coordinate system " << port[i]->getName() <<":" << endl;
      outputfile << ASK[i] << endl << endl;
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      outputfile << "# Translation of contour " << contour[i]->getName() <<":" << endl;
      outputfile << SrSC[i] << endl << endl;
      outputfile << "# Rotation of contour " << contour[i]->getName() <<":" << endl;
      outputfile << ASC[i] << endl << endl;
    }

    outputfile << "# Coordinate system for kinematics:" << endl;
    outputfile << port[iRef]->getName() << endl << endl;

    outputfile << "# Frame of Reference:" << endl;
    outputfile << portParent->getFullName() << endl << endl;

    if(fPrPK)
      fPrPK->save(path,outputfile);
    else
      outputfile << "# Type of translation:" << endl << endl;

    if(fAPK)
      fAPK->save(path,outputfile);
    else
      outputfile << "# Type of rotation:" << endl << endl;
   }

  void RigidBody::load(const string &path, ifstream& inputfile) {
    Body::load(path,inputfile);
    string dummy;

    getline(inputfile,dummy); // # Mass
    inputfile >> m;
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # Inertia tensor
    Mat buf;
    inputfile >> buf;
    getline(inputfile,dummy); // Rest of line
    getline(inputfile,dummy); // Newline
    SThetaS = SymMat(buf);
    i4I = 0;

    for(unsigned int i=1; i<port.size(); i++) {
      SrSK.push_back(Vec(3));
      WrSK.push_back(Vec(3));
      ASK.push_back(SqrMat(3));
      getline(inputfile,dummy); // # Translation cosy 
      inputfile >> SrSK[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // Newline
      getline(inputfile,dummy); // # Rotation cosy
      inputfile >> ASK[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // Newline
    }

    for(unsigned int i=0; i<contour.size(); i++) {
      SrSC.push_back(Vec(3));
      WrSC.push_back(Vec(3));
      ASC.push_back(SqrMat(3));
      getline(inputfile,dummy); // # Translation contour 
      inputfile >> SrSC[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // Newline
      getline(inputfile,dummy); // # Rotation contour
      inputfile >> ASC[i];
      getline(inputfile,dummy); // Rest of line
      getline(inputfile,dummy); // Newline
    }
    
    getline(inputfile,dummy); // # Coordinate system for kinematics
    getline(inputfile,dummy); // Coordinate system for kinematics
    setCoordinateSystemForKinematics(getCoordinateSystem(dummy));
    getline(inputfile,dummy); // Newline

    getline(inputfile,dummy); // # Frame of reference
    getline(inputfile,dummy); // Coordinate system for kinematics
    setFrameOfReference(getMultiBodySystem()->findCoordinateSystem(dummy));
    getline(inputfile,dummy); // Newline

    int s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of translation:
    getline(inputfile,dummy); // Type of translation 
    inputfile.seekg(s,ios::beg);
    ClassFactory cf;
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of translation 
      getline(inputfile,dummy); // End of line
    } else {
      setTranslation(cf.getTranslation(dummy));
      fPrPK->load(path, inputfile);
    }

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of rotation:
    getline(inputfile,dummy); // Type of rotation
    inputfile.seekg(s,ios::beg);
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of rotation 
      getline(inputfile,dummy); // End of line
    } else {
      setRotation(cf.getRotation(dummy));
      fAPK->load(path, inputfile);
    }

  }

}
