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
#include "body_rigid_rel.h"
#include "port.h"
#include "contour.h"
#include "link.h"
#include "tree.h"
#include "multi_body_system.h"

namespace MBSim {

  BodyRigidRel::BodyRigidRel(const string &name) : BodyRigid(name), lSize(6), lInd(0), tree(0), successor(0), precessor(0), APK(3), APK0(3), PrPK(3), PrPK0(3), KrOK(3), KvK(3), j(6), C(6) {

    APK0(0,0)=1.0;
    APK0(1,1)=1.0;
    APK0(2,2)=1.0;
  }

  void BodyRigidRel::calcSize() {
    BodyRigid::calcSize();

    setqInd(tree->getqSize());
    setuInd(tree->getuSize());
    setxInd(tree->getxSize());
    setlInd(tree->getlSize());
    tree->setqSize(tree->getqSize()+qSize);
    tree->setuSize(tree->getuSize()+uSize);
    tree->setxSize(tree->getxSize()+xSize);
    tree->setlSize(tree->getlSize()+lSize);
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->calcSize();
    }
  }

  void BodyRigidRel::setMbs(MultiBodySystem* mbs_) {
    Element::setMbs(mbs_);
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->setMbs(mbs);
    }
  }

  void BodyRigidRel::setTree(Tree* tree_) {
    tree = tree_;
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->setTree(tree);
    }
  }

  void BodyRigidRel::updateqRef() {
    q>>(tree->getq()(qInd,qInd+qSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateqRef();
    }
  }

  void BodyRigidRel::updatezdRef() {
    qd>>(tree->getqd()(qInd,qInd+qSize-1));
    ud>>(tree->getud()(uInd,uInd+uSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updatezdRef();
    }
  }

  void BodyRigidRel::updateqdRef() {
    qd>>(tree->getqd()(qInd,qInd+qSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateqdRef();
    }
  }

  void BodyRigidRel::updateuRef() {
    u>>(tree->getu()(uInd,uInd+uSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateuRef();
    }
  }

  void BodyRigidRel::updatehRef() {
    h>>(tree->geth()(uInd,uInd+uSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updatehRef();
    }
  }

  void BodyRigidRel::updaterRef() {
    r>>(tree->getr()(uInd,uInd+uSize-1));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updaterRef();
    }
  }

  void BodyRigidRel::updateMhRef() {
    Index I = getlIndex();
    Mh>>tree->getMh()(I);
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateMhRef();
    }
  }

  void BodyRigidRel::updatelRef() {
    l>>(tree->getl()(Il));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updatelRef();
    }
  }

  void BodyRigidRel::updateJRef() {
    J>>(tree->getJ()(Il,Index(0,uInd+uSize-1)));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateJRef();
    }
  }

  void BodyRigidRel::updateTRef() {
    Index Iu = getuIndex();
    Index Iq = Index(qInd,qInd+qSize-1);
    T>>(tree->getT()(Iq,Iu));
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateTRef();
    }
  }

  void BodyRigidRel::initStage1() {
    BodyRigid::init();

    IuT = Index(uInd+iT.start(),uInd+iT.end());
    IuR = Index(uInd+iR.start(),uInd+iR.end());

    vector<LinkPortData>::iterator it1=linkSetValuedPortData.begin(); 
    for(unsigned int i=0; i<linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      bool addLink = true;
      for(unsigned int j=0; j<tree->linkSetValuedPortData.size(); j++)
	if(tree->linkSetValuedPortData[j].link == it1->link)
	  addLink = false;
      if(addLink)
	tree->addLink(it1->link,port[portID],objectID);
      it1++;
    }
    vector<LinkPortData>::iterator it4=linkSingleValuedPortData.begin(); 
    for(unsigned int i=0; i<linkSingleValuedPortData.size(); i++) {
      int portID = it4->ID;
      int objectID = it4->objectID;
      bool addLink = true;
      for(unsigned int j=0; j<tree->linkSingleValuedPortData.size(); j++)
	if(tree->linkSingleValuedPortData[j].link == it4->link)
	  addLink = false;
      if(addLink)
	tree->addLink(it4->link,port[portID],objectID);
      it4++;
    }

    vector<LinkContourData>::iterator it2=linkSetValuedContourData.begin(); 
    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      int contourID = it2->ID;
      int objectID = it2->objectID;
      bool addLink = true;
      for(unsigned int j=0; j<tree->linkSetValuedContourData.size(); j++)
	if(tree->linkSetValuedContourData[j].link == it2->link)
	  addLink = false;
      if(addLink) 
	tree->addLink(it2->link,contour[contourID],objectID);

      it2++;
    }
    vector<LinkContourData>::iterator it3=linkSingleValuedContourData.begin(); 
    for(unsigned int i=0; i<linkSingleValuedContourData.size(); i++) {
      int contourID = it3->ID;
      int objectID = it3->objectID;
      bool addLink = true;
      for(unsigned int j=0; j<tree->linkSingleValuedContourData.size(); j++)
	if(tree->linkSingleValuedContourData[j].link == it3->link)
	  addLink = false;
      if(addLink) 
	tree->addLink(it3->link,contour[contourID],objectID);

      it3++;
    }

    for(unsigned int i=0;i<port.size();i++)
      tree->addPort(port[i]);			    // port und contour container von tree nur temporaere Liste 
    for(unsigned int i=0;i<contour.size();i++)      // z.b. zum Erstellen von ports2plot (multi_body_system.cc)
      tree->addContour(contour[i]);

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->initStage1();
    }
    iI = Index(0,uInd+uSize-1);
  }

  void BodyRigidRel::initStage2() {
    int k=0;
    for(unsigned int i=0; i<linkSetValued.size(); i++) {
      int linkID = -1;
      for(unsigned int j=0; j<tree->linkSetValued.size(); j++)
	if(tree->linkSetValued[j] == linkSetValued[i])
	  linkID = j;
      Index iJ(0,linkSetValued[i]->getlaSize()-1);
      if(linkID > -1) {
	W[i].resize() >> tree->W[linkID](iI,iJ);
	w[i].resize() >> tree->w[linkID](iJ);
      } else {
	W[i].resize() >> tree->W[k](iI,iJ);
	w[i].resize() >> tree->w[k](iJ);
	k++;
      }
    }
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->initStage2();
    }
  }


  void BodyRigidRel::initz() {
    Body::initz();
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->initz();
    }
  }

  void BodyRigidRel::initPlotFiles() {

    BodyRigid::initPlotFiles();

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->initPlotFiles();
    }
  }

  void BodyRigidRel::plotParameterFiles() {
    BodyRigid::plotParameterFiles(); 
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->plotParameterFiles();
    }
  }

  void BodyRigidRel::plot(double t, double dt) {
    BodyRigid::plot(t); 

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->plot(t,dt);
    }
  }

  void BodyRigidRel::updateFullName() {
    Element::setFullName(tree->getFullName()+"."+getName());
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateFullName();
    }
  } 

  void BodyRigidRel::addChild(BodyRigidRel *body) {

    successor.push_back(body);
    body->setMbs(mbs);
    body->setTree(tree);
    body->updateFullName();
    body->setPrecessor(this);
  }

  void BodyRigidRel::setPrecessor(BodyRigidRel *precessor_) {
    precessor = precessor_;
  }

  void BodyRigidRel::updateM(double t) {
    tree->getM()(Index(0,uInd+uSize-1)) += JTMJ(Mh,J);
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateM(t);
    }
  }
 
  void BodyRigidRel::updateh(double t) {
    sumUpForceElements(t);
    Vec KF = trans(AWK)*WF;
    Vec KM = trans(AWK)*WM;
    l(0,2) = KF - m*crossProduct(KomegaK,crossProduct(KomegaK,KrKS));
    l(3,5) = KM + crossProduct(I*KomegaK,KomegaK);

    if(precessor) {
      C(Index(0,2),Index(0,2)) = trans(APK);
      C(Index(3,5),Index(3,5)) = trans(APK);
      C(Index(0,2),Index(3,5)) = -trans(APK)*tilde(PrPK);

      Vec f(6,NONINIT);
      f(0,2) = trans(APK)*(crossProduct(precessor->getKomegaK(), 2*(JT*u(iT))+crossProduct(precessor->getKomegaK(),PrPK)));
      f(3,5) = crossProduct(KomegaK,JR*u(iR));

      j = C*precessor->getj()+f;

      l -= Mh*j;

//      BodyRigidRel* nextBody = precessor;
//      while(nextBody) {
//	J(Index(0,5),Index(nextBody->getIuT().start(),nextBody->getIuR().end())) = C*precessor->getJ()(Index(0,5),Index(nextBody->getIuT().start(),nextBody->getIuR().end()));
//	nextBody = nextBody->getPrecessor();
//      }
//
// TODO: ist nur eine voruebergehende Lsg, bis mir was besseres einfaellt...
       J(Index(0,5),Index(0,precessor->getIuR().end())) = C*precessor->getJ();
// ...!

    }
    J(Index(0,2),IuT) = trans(APK)*JT;
    J(Index(3,5),IuR) = JR;

//cout << getFullName() << endl << J << endl;

    tree->geth()(Index(0,uInd+uSize-1)) += trans(J)*l;

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateh(t);
    }
//    if(successor.size()==0) throw 1;
  }

  void BodyRigidRel::updateCenterOfGravity(double t) {

    (this->*updateAK0K)();

    PrPK = JT*q(iT) + PrPK0;
    APK = APK0*AK0K;
    if(precessor) {
      AWK = precessor->getAWK()*APK;
      KomegaK = trans(APK)*precessor->getKomegaK() + JR*u(iR);
      WomegaK = AWK * KomegaK;
      KrOK = trans(APK)*(PrPK + precessor->getKrOK());
      KvK = trans(APK)*(precessor->getKvK() + JT*u(iT) + crossProduct(precessor->getKomegaK(),PrPK));
    } else {
      AWK = APK;
      KomegaK = JR*u(iR);
      WomegaK = AWK * KomegaK;
      KrOK = trans(APK)*(PrPK);
      KvK = trans(APK)*JT*u(iT);
    }

    WrOK = AWK * KrOK;
    WvK = AWK * KvK;
  }

  void BodyRigidRel::updateKinematics(double t) {

    BodyRigid::updateKinematics(t);

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateKinematics(t);
    }
  }

  void BodyRigidRel::updateWj(double t) {
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
      Mat Kl(6,ld.cols(),NONINIT);
      Kl(IF,iJ) = trans(AWK)*ld(IF,iJ);
      Kl(IM,iJ) = trans(AWK)*ld(IM,iJ) + tilde(KrKP[portID])*Kl(IF,iJ) ;
      (*itW) += trans(J)*Kl;
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
	(*itW) += trans(J)*Kl;
      }
      it2++; itW++; 
    }
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateWj(t);
    }
  }

  void BodyRigidRel::updatewj(double t) {

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
      Mat Wl(6,ld.cols(),NONINIT);
      Wl(IF,iJ) = ld(IF,iJ);
      Wl(IM,iJ) = ld(IM,iJ) + tilde(WrKP[portID])*Wl(IF,iJ) ;
      Vec Wjs(6,NONINIT);
      Wjs(IF) = AWK*j(IF)+crossProduct(WomegaK,crossProduct(WomegaK,WrKP[portID])); 
      Wjs(IM) = AWK*j(IM);
      (*itw) += it1->link->getw(objectID) + trans(Wl) * Wjs; 
      it1++; itw++;
    }

    for(unsigned int i=0; i<linkSetValuedContourData.size(); i++) {
      if(it2->link->isActive()) {
	int objectID = it2->objectID;
	Mat ld = it2->link->getLoadDirections(objectID);
	Vec WrKC = it2->link->getWrOC(objectID)-WrOK;
	Index iJ(0,ld.cols()-1);
	cout << "Error: no implementation of updatew for contacts in BodyRigidRel yet. Use Time-Stepping-Integrator instead of ODE-Integrator." << endl;
	// (*itw) += 
      }
      it2++; itw++;
    }
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updatewj(t);
    }
  }

  void BodyRigidRel::updater(double t) {
    for(vector<LinkPortData>::iterator i=linkSetValuedPortData.begin(); i!=linkSetValuedPortData.end(); i++) {
      Index I = i->link->getlaIndex();
      tree->getr() += mbs->getW()(tree->getuIndex(),I)*i->link->getla();
    }
    for(vector<LinkContourData>::iterator i=linkSetValuedContourData.begin(); i!=linkSetValuedContourData.end(); i++) {
      if(i->link->isActive()) {
	Index I = i->link->getlaIndex();
	tree->getr() += mbs->getW()(tree->getuIndex(),I)*i->link->getla();
      }
    }
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updater(t);
    }
  }

  double BodyRigidRel::computeKineticEnergy() {
    return 0.5 * (m*trans(KvK)*(KvK + 2*crossProduct(KomegaK,KrKS)) + trans(KomegaK)*I*KomegaK);
  }
  
  double BodyRigidRel::computeKineticEnergyBranch() {
    double Ttemp = computeKineticEnergy();
    for(unsigned int i=0; i<successor.size(); i++)
      Ttemp += successor[i]->computeKineticEnergyBranch();
    return Ttemp;
  }

  double BodyRigidRel::computePotentialEnergyBranch() {
    double Vbranch = this->computePotentialEnergy();
    for(unsigned int i=0; i<successor.size(); i++)
      Vbranch += successor[i]->computePotentialEnergy();
    return Vbranch;
  }

  void BodyRigidRel::updatedq(double t, double dt) {

    BodyRigid::updatedq(t,dt);

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updatedq(t,dt);
    }
  }

  void BodyRigidRel::updateqd(double t) {
    qd = T*u;
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateqd(t);
    }
  }

  Port* BodyRigidRel::getPort(const string &pName) {
     // Auf sich selber suchen
    for(unsigned int i=0;i<port.size();i++)
      if(port[i]->getName() == pName)
        return port[i];
    for(unsigned int i=0;i<successor.size();i++) {
      Port* p = successor[i]->getPort(pName);
      if(p) return p;
    }
    return NULL;
  }

  Contour* BodyRigidRel::getContour(const string &cName) {
     // Auf sich selber suchen
    for(unsigned int i=0;i<contour.size();i++)
      if(contour[i]->getName() == cName)
        return contour[i];
    for(unsigned int i=0;i<successor.size();i++) {
      Contour* c = successor[i]->getContour(cName);
      if(c) return c;
    }
    return NULL;
  }

  void BodyRigidRel::addPort(Port * port, const Vec &KrKP) {		 
    BodyRigid::addPort(port, KrKP);
  }

  void BodyRigidRel::addPort(const string &name, const Vec &KrKP) {
    Port *port = new Port(name);
    addPort(port, KrKP);
  }

  void BodyRigidRel::addContour(Contour* contour, const Vec &KrKC_, const SqrMat &AKC_) {
    BodyRigid::addContour(contour, KrKC_, AKC_);
  }

  void BodyRigidRel::updateKLC(double t, double dt) {

    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->updateKLC(t,dt);
    }

    Vec KLC = l;

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

    KLC = Mh*J(Index(0,5),Index(0,getIuR().end()))*tree->getud()(0,getIuR().end())/dt-KLC;

    for(unsigned int i=0; i<successor.size(); i++) {
      KLC += trans(successor[i]->getC())*successor[i]->getl();
    }
  }

  void BodyRigidRel::plotNameToStream(ostream& os, string indent) {
	os << indent << getName() << endl;
    for(unsigned int i=0; i<successor.size(); i++) {
      successor[i]->plotNameToStream(os,indent+"  ");
    }
  }

}
