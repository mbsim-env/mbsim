/* Copyright (C) 2004-2006  Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */
#include <config.h>

#include "body_rigid_rel_onflex.h"

#include "port.h"
#include "contour.h"
#include "link.h"
#include "multi_body_system.h"
#include "tree_rigid.h"
#include "tree_flexroot.h"
#include "body_flexible.h"

namespace MBSim {

  BodyRigidRelOnFlex::BodyRigidRelOnFlex(const string &name) : BodyRigidRel(name), precessor(0), constcPosition(false) {
    cPosition.type = CONTINUUM;
  }

  void BodyRigidRelOnFlex::initStage2() {
    BodyRigidRel::initStage2();
    Iflexible = static_cast<TreeFlexRoot*>(tree)->Iflexible;
    C = Mat(6,precessor->JT.cols()+precessor->JR.cols(),INIT,0.0);
    preIJT = Index(                   0,precessor->JT.cols()                     -1);
    preIJR = Index(precessor->JT.cols(),precessor->JT.cols()+precessor->JR.cols()-1);
  }

  void BodyRigidRelOnFlex::sets0(const Vec& s0_) {
    constcPosition = true;
    cPosition.alpha = s0_;
    cPosition.alphap = Vec(0);
  }

  void BodyRigidRelOnFlex::updateqRef() {
    BodyRigidRel::updateqRef();
    if(!constcPosition) cPosition.alpha  >> q(iT);
  }
  void BodyRigidRelOnFlex::updateuRef() {
    BodyRigidRel::updateuRef();
    if(!constcPosition) cPosition.alphap >> u(iT);
  }
 
  void BodyRigidRelOnFlex::updateM(double t) {
    static Index AllCartesian(0,5);
    
    SymMat MTree  = tree->getM();

    MTree(Iflexible)    += JTMJ(Mh,J(AllCartesian,Iflexible));
    MTree(Iu,Iflexible) += trans(J(AllCartesian,Iu))*Mh*J(AllCartesian,Iflexible);
    MTree(      Iu)     += JTMJ(Mh,J(AllCartesian,Iu));

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateM(t);
    }
  }
 
  void BodyRigidRelOnFlex::updateh(double t) {
    sumUpForceElements(t);

//    Vec KF = trans(AWK)*WF;
//    Vec KM = trans(AWK)*WM;
//    l(0,2) = KF - m*crossProduct(KomegaK,crossProduct(KomegaK,KrKS));
//    l(3,5) = KM + crossProduct(I*KomegaK,KomegaK);
    l(0,2) = trans(AWK)*WF - m*crossProduct(KomegaK,crossProduct(KomegaK,KrKS));
    l(3,5) = trans(AWK)*WM + crossProduct(I*KomegaK,KomegaK);

    static Vec f(6,NONINIT);
    static Index AllCartesian(0,5);

// ---  KomegaK  = trans(AWK)*precessor->computeWomega(cp) + JR*u(iR);
// ---           = trans(AWK)*precessor->computeJacobianMatrix(iR)*precessor->u + JR*u(iR);
// ---  WomegaK  = precessor->computeWomega(cp) + AWK*JR*u(iR);

//       SqrMat AWKp = precessor->computeAWKp(cp)*APK  +  AWP*tilde(APK*JR*u(iR))*APK;
//    SqrMat AWKp = tilde(WomegaK)*AWK;
//    f(3,5) = trans(AWK)*AWKp*JR*u(iR);

//    if(JR.cols()) f(3,5) = trans(AWK)*tilde(WomegaK)*AWK*JR*u(iR); // TODO: Pruefen,
//    ob untere Zeile wirklich richtig!!!!!!!11
    if(JR.cols()) f(3,5) = tilde(KomegaK)*JR*u(iR);
    else f(3,5).init(0.0);

    if(JT.cols()) {
      f(3,5) += trans(AWK)*precessor->computeKp(cPosition) *u(iT);

// ---   WrOK = precessor->computeWrOC(cp);
// ---   WvK  = precessor->computeWvC(cp) + precessor->computeDrDs(cp)*u(iT);
// ---   WaK  = precessor->computeWaC(cp) + precessor->computeDrDs(cp)*up(iT) + precessor->computeDrDsp(cp)*u(iT);
      f(0,2)  = trans(AWK)*precessor->computeDrDsp(cPosition)*u(iT);
    }
    else f(0,2).init(0.0);

    C(Index(0,2),preIJT) = trans(AWK)*precessor->JT;
    C(Index(3,5),preIJR) = trans(AWK)*precessor->JR;

//      e = C*precessor->gete()+f;
////    e = C*Jges_pre*trans(precessor->computeJp(cPosition))*precessor->getu() + f;
    j = C*trans(precessor->computeJp(cPosition))*precessor->getu() + f;

//      J(Index(0,2),IuT) = trans(APK)*JT;
    if(JT.cols()) {
      J(Index(0,2),IuT) = trans(AWK)*precessor->computeDrDs(cPosition);
      J(Index(3,5),IuT) = trans(AWK)*precessor->computeK(cPosition);
    }
    J(Index(3,5),IuR) = JR;

////    J(Index(0,5),static_cast<TreeFlexRoot*>(tree)->Iflexible) = C*Jges_pre*trans(precessor->computeJacobianMatrix(cPosition));
    J(Index(0,5),Iflexible) = C*trans(precessor->computeJacobianMatrix(cPosition));

    l -= Mh*j;
//    tree->geth()(Index(0,uInd+uSize-1)) += trans(J)*l;
    Vec hTree = tree->geth();
    hTree(Iflexible) += trans(J(AllCartesian,Iflexible))*l;
    hTree(Iu       ) += trans(J(AllCartesian,Iu))       *l;

    for(unsigned int i=0; i<successor.size(); i++)
      successor[i]->updateh(t);
  }

  void BodyRigidRelOnFlex::updateWj(double t) {
    static Index IF(0,2);
    static Index IM(3,5);
    static Index AllCartesian(0,5);

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    vector<Mat>::iterator itW=W.begin(); 
    for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      Mat ld = it1->link->getLoadDirections(objectID);
      Index iJ(0,ld.cols()-1);
      Mat Kl(6,ld.cols(),NONINIT);
      Kl(IF,iJ) = trans(AWK)*ld(IF,iJ);
      Kl(IM,iJ) = trans(AWK)*ld(IM,iJ) + tilde(KrKP[portID])*Kl(IF,iJ);
      Mat W = (*itW);
      W(Iflexible,Index(0,ld.cols()-1)) += trans(J(AllCartesian,Iflexible))*Kl;
      W(Iu       ,Index(0,ld.cols()-1)) += trans(J(AllCartesian,Iu       ))*Kl;
      it1++; itW++; 
    }

    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	int objectID = it2->objectID;
	Mat ld = it2->link->getLoadDirections(objectID);
	Index iJ(0,ld.cols()-1);
	Mat Kl(6,ld.cols(),NONINIT);
	Vec KrKC = trans(AWK)*(it2->link->getWrOC(objectID)-WrOK);
	Kl(IF,iJ) = trans(AWK)*ld(IF,iJ);
	Kl(IM,iJ) = trans(AWK)*ld(IM,iJ) + tilde(KrKC)*Kl(IF,iJ) ;
	Mat W = (*itW);
	W(Iflexible,Index(0,ld.cols()-1)) += trans(J(AllCartesian,Iflexible))*Kl;
	W(Iu       ,Index(0,ld.cols()-1)) += trans(J(AllCartesian,Iu       ))*Kl;
      }
      it2++; itW++; 
    }
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateWj(t);
    }
  }

  void BodyRigidRelOnFlex::updateCenterOfGravity(double t) {
    (this->*updateAK0K)();

    //    PrPK = JT*q(iT) + PrPK0;
    APK = APK0*AK0K;
//    SqrMat AWP = precessor->computeAWK(cPosition);
//    AWK = AWP*APK;
    AWK = precessor->computeAWK(cPosition)*APK;

    //    KomegaK = trans(APK)*precessor->getKomegaK() + JR*u(iR);
    KomegaK = trans(AWK)*precessor->computeWomega(cPosition) + JR*u(iR);
    //cout << "-----------------" << endl;
    //cout << "KomegaK ohne K " << trans(KomegaK) << endl;
    // precessor many have curved path, leading to angular processes when translating 
    if(JT.cols()) {
      KomegaK += trans(AWK)*precessor->computeK(cPosition) * u(iT);   
      //cout << "KomegaK mit  K " << trans(KomegaK) << endl;
      //cout << "precessor->computeK(cp)" << trans(precessor->computeK(cp)) << endl;
      //cout << "u(iT) " << trans(u(iT)) << endl;
      //cout << "+= trans(AWK)*precessor->computeK(cp) * u(iT) " << trans(trans(AWK)*precessor->computeK(cp) * u(iT)) << endl;
    }
    WomegaK = AWK * KomegaK;

    //      KrOK = trans(APK)*(PrPK + precessor->getKrOK());
    WrOK = precessor->computeWrOC(cPosition);
    //      WvK  = precessor->computeWvC(cp) + AWP*JT*u(iT);
    WvK  = precessor->computeWvC(cPosition);
    if(JT.cols())
      WvK += precessor->computeDrDs(cPosition)*u(iT);
  }

   const Vec& BodyRigidRelOnFlex::getKrOK()  {
    KrOK = trans(AWK) * WrOK;
    return KrOK;
  }

   const Vec& BodyRigidRelOnFlex::getKvK()   {
    KvK = trans(AWK) * WvK;
    return KvK;
  }
}
