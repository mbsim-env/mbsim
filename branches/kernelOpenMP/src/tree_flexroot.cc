/* Copyright (C) 2007 Roland Zander
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
 * Contact:
 *    rzander@users.berlios.de
 */

#include <config.h>

#include "tree_flexroot.h"

#include "multi_body_system.h"
#include "body_flexible.h"
#include "body_rigid_rel_onflex.h"
///////////////////////////
#include <omp.h>
unsigned long int timemeasureepoch();
///////////////////////////

namespace MBSim {
  TreeFlexRoot::TreeFlexRoot(const string &projectName) : Tree(projectName), flexible(NULL) {
  }

  TreeFlexRoot::~TreeFlexRoot() {
    delete flexible;
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      delete (*ib); 
  }

  Body* TreeFlexRoot::getRoot() {return flexible;}

  void TreeFlexRoot::updateKinematics(double t) {
    flexible->updateKinematics(t);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateKinematics(t); 
  }

  void TreeFlexRoot::updateT(double t) { 
    flexible->updateT(t);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateT(t);
  }

//  void TreeFlexRoot::resetMh() {
//    M(Iflexible,Irigid).init(0.0);
//    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
//      (*ib)->getM().init(0.0);
//    h(Irigid)      .init(0.0);
//  }

  void TreeFlexRoot::updateh(double t) {
    M.init(0.0);
    h.init(0.0);
    flexible->updateh(t);  // beschreibt durch updatehRef(...) auf die passenden Speicherbereiche
			   // update auch fuer zugehoeriges M()
    //    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib) {
    //    for(unsigned int i=0; i < rigid.size(); i++)
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateh(t);
  }

  void TreeFlexRoot::updateM(double t) {
    flexible->updateM(t);
///////////////////
fprintf(stderr,"%ld %d\n",timemeasureepoch(),1005000);
#   pragma omp parallel for
    for(int i=0; i<(int)rigid.size(); i++) {
fprintf(stderr,"%ld %d\n",timemeasureepoch(),1006001+omp_get_thread_num());
      rigid[i]->updateM(t);
fprintf(stderr,"%ld %d\n",timemeasureepoch(),1007001+omp_get_thread_num());
    }
fprintf(stderr,"%ld %d\n",timemeasureepoch(),1004000);
///////////////////
//#   pragma omp parallel for
//    for(int i=0; i<(int)rigid.size(); i++)
//      rigid[i]->updateM(t);
////////////////////////////
  }


  void TreeFlexRoot::updateWj(double t) {
    for(unsigned int i=0; i<W.size(); i++)
      W[i].init(0);
 
    flexible->updateWj(t);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateWj(t);
  }

  void TreeFlexRoot::updatedq(double t, double dt) {
    flexible->updatedq(t,dt);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updatedq(t,dt);
  }

  void TreeFlexRoot::updatedu(double t, double dt) {
    ud = slvLLFac(LLM, h*dt+r);
  }

//  void TreeFlexRoot::updateqd(double t) {
//    flexible->updateqd(t);
//    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
//      (*ib)->updateqd(t);
//  }


  void TreeFlexRoot::updatezd(double t) {
    flexible->updateqd(t);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateqd(t);
    ud = slvLLFac(LLM, h+r);
  }

  void TreeFlexRoot::updateqRef() {
    Object::updateqRef();
    flexible->updateqRef(q(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateqRef();
  }

  void TreeFlexRoot::updateqdRef() {
    Object::updateqdRef();
    flexible->updateqdRef(qd(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateqdRef();
  }

  void TreeFlexRoot::updatezRef() {
    Object::updatezRef();
    flexible->updateqRef(q(Iflexible));
    flexible->updateuRef(u(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updatezRef();
  }

  void TreeFlexRoot::updatezdRef() {
    Object::updatezdRef();
    flexible->updateqdRef(qd(Iflexible));
    flexible->updateudRef(ud(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updatezdRef();
  }

  void TreeFlexRoot::updateuRef() {
    Object::updateuRef();
    flexible->updateuRef(u(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateuRef();
  }

  void TreeFlexRoot::updateMRef() {
    Object::updateMRef();
    flexible->updateMRef(M(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateMRef();
  }

  void TreeFlexRoot::updatehRef() {
    Object::updatehRef();
    flexible->updatehRef(h(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updatehRef();
  }

  void TreeFlexRoot::updaterRef() {
    Object::updaterRef();
    flexible->updaterRef(r(Iflexible));
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updaterRef();
  }

  void TreeFlexRoot::updateTRef() {

    Object::updateTRef();
    flexible->updateTRef();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->updateTRef();
  }

  void TreeFlexRoot::calcSize() {
    flexible->calcSize();
    flexible->setqInd(qSize);
    flexible->setuInd(uSize);
    qSize = flexible->getqSize();
    uSize = flexible->getuSize();
    lSize = qSize;
    Iflexible = Index(0,uSize-1);

    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->calcSize();

    J.resize(getlSize(),getuSize());
    Mh.resize(getlSize());
    l.resize(getlSize());

    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib) {
      (*ib)->updatelRef();
      (*ib)->updateJRef();
      (*ib)->updateMhRef();
    }
    Irigid = Index(flexible->getuSize(),uSize-1);
  }

  void TreeFlexRoot::init() {
    J.init(0.0);
    M.init(0.0);
    h.init(0.0);

    initFlexibleStage1();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib) {
      (*ib)->setPrecessor(flexible);
      (*ib)->initStage1();
    }

    Object::init();
    flexible->init();

    initFlexibleStage2();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->initStage2();
  }



  void TreeFlexRoot::initFlexibleStage1() {
    vector<LinkPortData>::iterator it1=flexible->linkSetValuedPortData.begin(); 
    for(unsigned int i=0; i<flexible->linkSetValuedPortData.size(); i++) {
      int portID = it1->ID;
      int objectID = it1->objectID;
      bool bAddLink = true;
      for(unsigned int j=0; j<linkSetValuedPortData.size(); j++)
	if(linkSetValuedPortData[j].link == it1->link)
	  bAddLink = false;
      if(bAddLink)
	addLink(it1->link,flexible->port[portID],objectID);
      it1++;
    }

    vector<LinkContourData>::iterator it2=flexible->linkSetValuedContourData.begin(); 
    for(unsigned int i=0; i<flexible->linkSetValuedContourData.size(); i++) {
      int contourID = it2->ID;
      int objectID = it2->objectID;
      bool bAddLink = true;
      for(unsigned int j=0; j<linkSetValuedContourData.size(); j++)
	if(linkSetValuedContourData[j].link == it2->link)
	  bAddLink = false;
      if(bAddLink)
	addLink(it2->link,flexible->contour[contourID],objectID);
      it2++;
    }
  }

  void TreeFlexRoot::initFlexibleStage2() {
	int k=0;
	for(unsigned int i=0; i<flexible->linkSetValued.size(); i++) {
	  int linkID = -1;
	  for(unsigned int j=0; j<linkSetValued.size(); j++)
		if(linkSetValued[j] == flexible->linkSetValued[i]) // ?!?!?!? was ist denn nun das: 16.06.2008, auf Roberts Rat hin flexible->...[i]
		  linkID = j;
	  Index iJ(0,flexible->linkSetValued[i]->getlaSize()-1);
	  if(linkID > -1) {
		flexible->W[i].resize() >> W[linkID](Iflexible,iJ);
	  } else {
		flexible->W[i].resize() >> W[k](Iflexible,iJ);
		k++;
	  }
	}
  }


  void TreeFlexRoot::initz() {
    Object::initz();
    flexible->initz();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->initz();
  }

  void TreeFlexRoot::initPlotFiles() {
    Object::initPlotFiles();
    flexible->initPlotFiles();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib){
      (*ib)->initPlotFiles();
}
  }

  void TreeFlexRoot::plot(double t, double dt) {
    Object::plot(t,dt);
    flexible->plot(t,dt);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->plot(t,dt);
  }

  void TreeFlexRoot::setBodyFlexible(BodyFlexible* flexible_) {
    flexible = flexible_;
    flexible->setMbs(mbs);
    flexible->setFullName(fullName  + "." + flexible->getName());
  }

  void TreeFlexRoot::addBodyRigidRelOnFlex(BodyRigidRelOnFlex* bodyAdd, Vec s0_) {
    if(s0_.size() == 0)
      bodyAdd->setJT(Vec("[1.0;0.0;0.0]"));
    else
      bodyAdd->sets0(s0_);

    bodyAdd->setMbs(mbs);
    bodyAdd->setTree(this);
    bodyAdd->setFullName(fullName  + "." + bodyAdd->getName());
    rigid.push_back(bodyAdd);
  } 

  void TreeFlexRoot::setFullName(const string &name_) {
    fullName = name_;
    if(flexible) flexible->setFullName(fullName  + "." + flexible->getName());
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->setFullName(fullName  + "." + flexible->getName());
  }

  void TreeFlexRoot::setMbs(MultiBodySystem* mbs_) {
    Element::setMbs(mbs_);
    if(flexible) flexible->setMbs(mbs);
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->setMbs(mbs);
  }

  double TreeFlexRoot::computePotentialEnergy() {
    double Vtemp = flexible->computePotentialEnergy();
    for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      Vtemp += (*ib)->computePotentialEnergyBranch();
    return Vtemp;
  }

  void TreeFlexRoot::plotParameterFiles() {
	Tree::plotParameterFiles();
	for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
      (*ib)->plotParameterFiles();
  }

  void TreeFlexRoot::plotParameters() {
	parafile << "TreeFlexRoot" << endl<<endl;
	parafile << "# BodyList:" << endl;
	parafile << flexible->getName() << endl;
	for(vector<BodyRigidRelOnFlex*>::iterator ib = rigid.begin(); ib != rigid.end(); ++ib)
	  (*ib)->plotNameToStream(parafile,"  ");
  }

}
