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
#include "tree.h"
#include "multi_body_system.h"
#include "coordinate_system.h"

namespace MBSim {

  Tree::Tree(const string &projectName) : Subsystem(projectName) {
  }

  Tree::~Tree() {
  }

  void Tree::updateKinematics(double t) {

    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateKinematics(t);
  }

  void Tree::updateT(double t) { 
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateT(t);
  }

  void Tree::updateh(double t) {
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateh(t);
  }

  void Tree::updateM(double t) {
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateM(t);
  }

  void Tree::updateqRef() {

    Object::updateqRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateqRef();
  }

  void Tree::updateMRef() {
    Object::updateMRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateMRef();
  }
  void Tree::updateLLMRef() {
    Object::updateLLMRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateLLMRef();
  }

  void Tree::updateqdRef() {

    Object::updateqdRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateqdRef();
  }

  void Tree::updatezdRef() {

    Object::updatezdRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updatezdRef();
  }

  void Tree::updateuRef() {

    Object::updateuRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateuRef();
  }

  void Tree::updatehRef() {

    Object::updatehRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updatehRef();
  }

  void Tree::updaterRef() {

    Object::updaterRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updaterRef();
  }

  void Tree::updateTRef() {

    Object::updateTRef();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->updateTRef();
  }

  void Tree::calchSize() {

  //  for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
  //    (*i)->sethSize(hSize);
  //    (*i)->calchSize();
  //  }
    for(unsigned i=0; i<object.size(); i++) {
      int j = object.size()-1-i;
      if(i==0)
        object[j]->sethSize(hSize);
      else
        object[j]->sethSize(object[j+1]->gethSize() - object[j+1]->getuSize());
      object[j]->calchSize();
    }
  }

  // void Tree::setMbs(MultiBodySystem* mbs_) {
  //   //Element::setMbs(mbs_);
  //   for(unsigned i=0; i<object.size(); i++)
  //     object[i]->setMbs(mbs);
  // }

  void Tree::init() {
    for(unsigned i=0; i<object.size(); i++)
      object[i]->init();

    Object::init();
  }

  void Tree::initz() {
    Object::initz();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initz();
  }

  void Tree::initPlotFiles() {
    Object::initPlotFiles();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initPlotFiles();
  }

  void Tree::plot(double t, double dt) {
    Object::plot(t,dt);
    for(unsigned i=0; i<object.size(); i++)
      object[i]->plot(t,dt);
  }

  void Tree::setFullName(const string &name) {
    Element::setFullName(name);
    for(unsigned i = 0; i<object.size(); i++) {
      object[i]->setFullName(getFullName()+"."+object[i]->getFullName());
    }
  }

  void Tree::addObject(Object* obj) {
    obj->setParent(this);
    object.push_back(obj);
    //obj->setMbs(mbs);
    obj->setFullName(getFullName()+"."+obj->getFullName());
  } 

  double Tree::computePotentialEnergy() {
    // double Vtemp = 0.0;
    // if(root) {
    //   Vtemp = computePotentialEnergyBranch(root);
    // }
    // return Vtemp;
    return -1;
  }

  //double Tree::computePotentialEnergyBranch(Object* body) {
  //  double Vtemp = 0.0;

  //  // Koerper selbst
  //  Vtemp += body->computePotentialEnergy();
  //  //    cout << body->getFullName() << endl;

  //  // ... und seine Nachfolger
  //  for(unsigned int i=0; i<body->successor.size(); i++) {
  //    Vtemp += computePotentialEnergyBranch(body->successor[i]);
  //  }
  //  return Vtemp;
  //}

}
