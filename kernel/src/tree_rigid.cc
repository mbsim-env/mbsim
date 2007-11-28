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
#include "tree_rigid.h"
#include "body_rigid_rel.h"
#include "multi_body_system.h"
#include "port.h"

namespace MBSim {

  TreeRigid::TreeRigid(const string &projectName) : Tree(projectName), root(0) {
  }

  TreeRigid::~TreeRigid() {
    delete root; 
  }

  void TreeRigid::updateKinematics(double t) {

    root->updateKinematics(t);
  }

  void TreeRigid::updateT(double t) { 
    root->updateT(t); 
  }

  void TreeRigid::updateh(double t) {
    h.init(0);
    root->updateh(t);
  }

  void TreeRigid::updateM(double t) {
    M.init(0);
    root->updateM(t); 
  }

  void TreeRigid::updateWj(double t) {
    vector<Mat>::iterator itW=W.begin(); 
    for(unsigned int i=0; i<W.size(); i++)
      W[i].init(0);
    root->updateWj(t);
    LLM = facLL(M); // TODO evtl. in updateh berechnen -> slvLL
  }

  void TreeRigid::updatedq(double t, double dt) {

    root->updatedq(t,dt);
    //qd = u*dt;
  }

  void TreeRigid::updatedu(double t, double dt) {

    //ud = slvLL(M,h*dt+r);
    ud = slvLLFac(LLM, h*dt+r);
  }

  void TreeRigid::updatezd(double t) {

    root->updateqd(t);
    ud =  slvLLFac(LLM, h+r);
  }

  void TreeRigid::updateqRef() {

    Object::updateqRef();
    root->updateqRef();
  }

  void TreeRigid::updateqdRef() {

    Object::updateqdRef();
    root->updateqdRef();
  }

  void TreeRigid::updatezdRef() {

    Object::updatezdRef();
    root->updatezdRef();
  }

  void TreeRigid::updateuRef() {

    Object::updateuRef();
    root->updateuRef();
  }

  void TreeRigid::updatehRef() {

    Object::updatehRef();
    root->updatehRef();
  }

  void TreeRigid::updaterRef() {

    Object::updaterRef();
    root->updaterRef();
  }

  void TreeRigid::updateTRef() {

    Object::updateTRef();
    root->updateTRef();
  }

  void TreeRigid::calcSize() {

    root->calcSize();

    J.resize(getlSize(),getuSize());
    Mh.resize(getlSize());
    l.resize(getlSize());
    root->updatelRef();
    root->updateJRef();
    root->updateMhRef();
  }

  void TreeRigid::init() {
    root->initStage1();
    Object::init();
    root->initStage2();
  }

  void TreeRigid::initz() {
    Object::initz();
    root->initz();
  }

  void TreeRigid::initPlotFiles() {
    Object::initPlotFiles();
    root->initPlotFiles();
  }

  void TreeRigid::plot(double t, double dt) {
    Object::plot(t,dt);
    root->plot(t,dt);
  }

  void TreeRigid::setRoot(BodyRigidRel* root_) {
    root = root_;
    root->setTree(this);
    root->setMbs(mbs);
    root->updateFullName();
  } 

  void TreeRigid::setFullName(const string &name) {
    Element::setFullName(name);
    if(root)
      root->updateFullName();
  } 

  void TreeRigid::setMbs(MultiBodySystem* mbs_) {
    Element::setMbs(mbs_);
    if(root)
      root->setMbs(mbs);
  }

  double TreeRigid::computePotentialEnergy() {
    double Vtemp = 0.0;
    if(root) {
      Vtemp = computePotentialEnergyBranch(root);
    }
    return Vtemp;
  }

  double TreeRigid::computePotentialEnergyBranch(BodyRigidRel* body) {
    double Vtemp = 0.0;

    // Koerper selbst
    Vtemp += body->computePotentialEnergy();
    //    cout << body->getFullName() << endl;

    // ... und seine Nachfolger
    for(unsigned int i=0; i<body->successor.size(); i++) {
      Vtemp += computePotentialEnergyBranch(body->successor[i]);
    }
    return Vtemp;
  }

  Port* TreeRigid::getPort(const string &pName) {
    return root->getPort(pName);
  }

  Contour* TreeRigid::getContour(const string &cName) {
    return root->getContour(cName);
  }
}
