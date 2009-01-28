/* Copyright (C) 2004-2006  Martin Förg
 
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
#include "body_rigid_abs.h"
#include "port.h"
#include "contour.h"
#include "link.h"
#include "multi_body_system.h"

namespace MBSim {

  BodyRigidAbs::BodyRigidAbs(const string &name) : BodyRigid(name), WrOK0(3), AWK0(3) { 

    AWK0(0,0)=1.0;
    AWK0(1,1)=1.0;
    AWK0(2,2)=1.0;
  }

  const Vec& BodyRigidAbs::getWrOS0() const {
    cout << "Warning: getWrOS0 is a deprecated function and only valid, if KrKS = 0. Use getWrOK0 instead." << endl;
    return WrOK0;
  }

  void BodyRigidAbs::setWrOS0(const Vec &WrOK0_) {
    WrOK0 = WrOK0_; 
    WrOK = WrOK0; 
    cout << "Warning: setWrOS0 is a deprecated function and only valid, if KrKS = 0. Use setWrOK0 instead." << endl;
  } 

  void BodyRigidAbs::setWrOK0(const Vec &WrOK0_) {
    WrOK0 = WrOK0_; 
    WrOK = WrOK0; 
  } 

  void BodyRigidAbs::setAWK0(const SqrMat &AWK0_) {
    AWK0 = AWK0_;
  }

  void BodyRigidAbs::init() {
    BodyRigid::init();

    J.resize(6,uSize);
    J(Index(0,2),iT) = JT;
    J(Index(3,5),iR) = JR;
  
    WrOHitSphere >> WrOK;
 
    M = JTMJ(Mh,J);
    LLM = facLL(M);

    if(nrm2(KrKS) <= 1e-14) {
      updateM_ = &BodyRigidAbs::updateM1;
      updateh_ = &BodyRigidAbs::updateh1;
      facLLM_ = &BodyRigidAbs::facLLM1;
    } else {
      updateM_ = &BodyRigidAbs::updateM2;
      updateh_ = &BodyRigidAbs::updateh2;
      facLLM_ = &BodyRigidAbs::facLLM2;
    }
  }

  void BodyRigidAbs::initPlotFiles() {

    BodyRigid::initPlotFiles();
  }

  void BodyRigidAbs::plot(double t, double dt) {

    BodyRigid::plot(t); 
  }

  void BodyRigidAbs::updateCenterOfGravity(double t) {

    (this->*updateAK0K)();

    WrOK = JT*q(iT) + WrOK0;
    WvK = JT*u(iT);
    AWK = AWK0*AK0K;
    KomegaK = JR*u(iR);
    WomegaK = AWK*KomegaK;
  }

  void BodyRigidAbs::updateM2() {
    J(Index(0,2),iT) = trans(AWK)*JT;
    M = JTMJ(Mh,J);
  }

  void BodyRigidAbs::updateh(double t) {
    sumUpForceElements(t);
    (this->*updateh_)();
  }

  void BodyRigidAbs::updateh1() {
    h(iT) = trans(JT)*(WF);
    h(iR) = trans(JR)*(trans(AWK)*WM + crossProduct(I*KomegaK,KomegaK));
  }

  void BodyRigidAbs::updateh2() {
    h(iT) = trans(JT)*(WF - AWK*(m*crossProduct(KomegaK,crossProduct(KomegaK,KrKS))));
    h(iR) = trans(JR)*(trans(AWK)*WM + crossProduct(I*KomegaK,KomegaK));
  }

  void BodyRigidAbs::facLLM2() {
    LLM = facLL(M);
  }

  void BodyRigidAbs::updateWj(double t) {

    Index IF(0,2);
    Index IM(3,5);

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    vector<Mat>::iterator itW=W.begin(); 
    for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      Mat ld = it1->link->getLoadDirections(objectID);
      Index iJ(0,ld.cols()-1);
      (*itW).resize(uSize,ld.cols(),NONINIT);
      (*itW)(iT,iJ) = trans(JT)*ld(IF,iJ);
      (*itW)(iR,iJ) = trans(JR)*(trans(AWK)*(ld(IM,iJ)+tilde(WrKP[portID])*ld(IF,iJ)));
      it1++; itW++;
    }

    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	int objectID = it2->objectID;
	Mat ld = it2->link->getLoadDirections(objectID);
	Vec WrKC = it2->link->getWrOC(objectID)-WrOK;
	Index iJ(0,ld.cols()-1);
	(*itW).resize(uSize,ld.cols(),NONINIT);
	(*itW)(iT,iJ) = trans(JT)*ld(IF,iJ);
	(*itW)(iR,iJ) = trans(JR)*(trans(AWK)*(ld(IM,iJ)+tilde(WrKC)*ld(IF,iJ)));
      }
      it2++; itW++; 
    }
  }

  void BodyRigidAbs::updatewj(double t) {

    Index IF(0,2);
    Index IM(3,5);

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    vector<Vec>::iterator itw=w.begin(); 
    for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      Mat ld = it1->link->getLoadDirections(objectID);
      Index iJ(0,ld.cols()-1);
      *itw = trans(ld(IF,iJ))*crossProduct(WomegaK,crossProduct(WomegaK,WrKP[portID])) +  it1->link->getw(objectID);
      it1++; itw++;
    }

    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	int objectID = it2->objectID;
	Mat ld = it2->link->getLoadDirections(objectID);
	Vec WrKC = it2->link->getWrOC(objectID)-WrOK;
	Index iJ(0,ld.cols()-1);
	cout << "Error: no implementation of updatew for contacts yet. Use Time-Stepping-Integrator instead of ODE-Integrator." << endl;
      }
      it2++; itw++;
    }
  }

  void BodyRigidAbs::updatezd(double t) {
    qd = T*u;
    ud = slvLLFac(LLM, h+r);
  }

  void BodyRigidAbs::updatedu(double t, double dt) {
    ud = slvLLFac(LLM, h*dt +r);
  }

  void BodyRigidAbs::updateKLC(double t, double dt) {

    Vec KLC = l;
    Vec KF = trans(AWK)*WF;
    Vec KM = trans(AWK)*WM;
    KLC(0,2) = KF - m*crossProduct(KomegaK,crossProduct(KomegaK,KrKS));
    KLC(3,5) = KM + crossProduct(I*KomegaK,KomegaK);
    J(Index(0,2),iT) = trans(AWK)*JT;

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      WLtmp = it1->link->getLoadDirections(objectID)*it1->link->getla()/dt;
      KLC(0,2) += trans(AWK)*WFtmp;
      KLC(3,5) += trans(AWK)*(WMtmp + crossProduct(WrKP[portID],WFtmp));
      it1++;
    }

    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	int objectID = it2->objectID;
	WLtmp = it2->link->getLoadDirections(objectID)*it2->link->getla()/dt;
	KLC(0,2) += trans(AWK)*WFtmp;
	Vec WrKC = linkSetValuedContourData[i].link->getWrOC(objectID)-WrOK;
	KLC(3,5) += trans(AWK)*(WMtmp + crossProduct(WrKC,WFtmp));
      }
      it2++; 
    } 

    KLC = Mh*J*ud/dt-KLC;
  }
}
