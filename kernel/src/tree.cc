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
#include "tree.h"
#include "body.h"
#include "body_rigid_rel.h"
#include "multi_body_system.h"
#include "port.h"

namespace MBSim {

  Tree::Tree(const string &projectName) : Object(projectName), lSize(0), root(0) {
  }

  Tree::~Tree() { 
  }

  void Tree::updateKinematics(double t) {

    root->updateKinematics(t);
  }

  void Tree::updateh(double t) {

    root->updateh(t);
  }

  //void Tree::updater(double t) {

  //Ob
  //r.init(0);
  //root->updater(t);
  //}

  void Tree::updateW(double t) {
    vector<Mat>::iterator itW=W.begin(); 
    for(int i=0; i<W.size(); i++)
      W[i].init(0);
    root->updateW(t);
    LLM = facLL(M); // TODO evtl. in updateh berechnen -> slvLL
  }

  void Tree::updatedq(double t, double dt) {

    root->updatedq(t,dt);
    //qd = u*dt;
  }

  void Tree::updatedu(double t, double dt) {

    //ud = slvLL(M,h*dt+r);
    ud = slvLLFac(LLM, h*dt+r);
  }

  void Tree::updatezd(double t) {

    root->updateqd(t);
    ud = slvLL(M,h);
  }

  void Tree::updateqRef() {

    Object::updateqRef();
    root->updateqRef();
  }

  void Tree::updateqdRef() {

    Object::updateqdRef();
    root->updateqdRef();
  }

  void Tree::updatezdRef() {

    Object::updatezdRef();
    root->updatezdRef();
  }

  void Tree::updateuRef() {

    Object::updateuRef();
    root->updateuRef();
  }

  void Tree::updatehRef() {

    Object::updatehRef();
    root->updatehRef();
  }

  void Tree::updaterRef() {

    Object::updaterRef();
    root->updaterRef();
  }

  void Tree::updateTRef() {

    Object::updateTRef();
    root->updateTRef();
  }

  void Tree::calcSize() {

    root->calcSize();

    J.resize(getlSize(),getuSize());
    Mh.resize(getlSize());
    l.resize(getlSize());
    root->updatelRef();
    root->updateJRef();
    root->updateMhRef();
  }

  void Tree::init() {
    root->initStage1();
    Object::init();
    root->initStage2();
  }

  void Tree::initz() {
    Object::initz();
    root->initz();
  }

  void Tree::initPlotFiles() {
    Object::initPlotFiles();
    root->initPlotFiles();
  }

  void Tree::plot(double t, double dt) {
    Object::plot(t,dt);
    root->plot(t,dt);
  }

  void Tree::setRoot(BodyRigidRel* root_) {
    root = root_;
    root->setTree(this);
    root->setMbs(mbs);
    root->updateFullName();
  } 

  void Tree::setFullName(const string &name) {
    Element::setFullName(name);
    if(root)
      root->updateFullName();
  } 

  void Tree::setMbs(MultiBodySystem* mbs_) {
    Element::setMbs(mbs_);
    if(root)
      root->setMbs(mbs);
  }

  double Tree::computePotentialEnergy() {
    double Vtemp = 0.0;
    if(root) {
      Vtemp = computePotentialEnergyBranch(root);
    }
    return Vtemp;
  }

  double Tree::computePotentialEnergyBranch(BodyRigidRel* body) {
    double Vtemp = 0.0;

    // Koerper selbst
    Vtemp += body->computePotentialEnergy();
    //    cout << body->getFullName() << endl;

    // ... und seine Nachfolger
    for(int i=0; i<body->successor.size(); i++) {
      Vtemp += computePotentialEnergyBranch(body->successor[i]);
    }
    return Vtemp;
  }

  Port* Tree::getPort(const string &pName) {
    return root->getPort(pName);
  }
}
