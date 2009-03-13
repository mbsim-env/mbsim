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
#include "object.h"
#include "link.h"
#include "frame.h"
#include "extra_dynamic_interface.h"

namespace MBSim {

  void Node::addChild(Node* child_) {
    child.push_back(child_);
  }

  void Node::updateKinematics(double t) {
   obj->updateKinematics(t);
   for(unsigned int i=0; i<child.size(); i++)
     child[i]->updateKinematics(t);
  }

  void Node::updateJacobians(double t) {
   obj->updateJacobians(t);
   for(unsigned int i=0; i<child.size(); i++)
     child[i]->updateJacobians(t);
  }

  void Node::updateSecondJacobians(double t) {
   obj->updateSecondJacobians(t);
   for(unsigned int i=0; i<child.size(); i++)
     child[i]->updateSecondJacobians(t);
  }

  void Node::sethSize(int &hSize, int j) {

    for(int i=child.size()-1; i>=0; i--)
      child[i]->sethSize(hSize,j);

    obj->sethSize(hSize,j);
    hSize -= obj->getuSize(j);
  }

  void Node::calcqSize(int &qSize) {

    obj->calcqSize();
    obj->setqInd(qSize);
    qSize += obj->getqSize();

    for(unsigned int i=0; i<child.size(); i++)
      child[i]->calcqSize(qSize);
  }

  void Node::calcuSize(int &uSize, int j) {

    obj->calcuSize(j);
    obj->setuInd(uSize,j);
    uSize += obj->getuSize(j);

    for(unsigned int i=0; i<child.size(); i++)
      child[i]->calcuSize(uSize,j);
  }

  Tree::Tree(const string &projectName) : Subsystem(projectName) {
  }

  Tree::~Tree() {
  }

  Node* Tree::addObject(Node* tree, Object* obj) {

    Subsystem::addObject(obj);

    Node *node = new Node(obj);
    if(tree)
      tree->addChild(node);
    else
      root = node;
    return node;
  }

  Node* Tree::addSubsystem(Node* tree, Subsystem *sys, const Vec &RrRS, const SqrMat &ARS, const Frame* refFrame) {

    Subsystem::addSubsystem(sys);

    int i = 0;
    if(refFrame)
      i = portIndex(refFrame);

    IrOS.push_back(IrOK[i] + AIK[i]*RrRS);
    AIS.push_back(AIK[i]*ARS);

    Node *node = new Node(sys);
    if(tree)
      tree->addChild(node);
    else
      root = node;
    return node;
  }

  void Tree::calcqSize() {
    qSize = 0;
    root->calcqSize(qSize);
  }

  void Tree::calcuSize(int j) {
    uSize[j] = 0;
    root->calcuSize(uSize[j],j);
  }

  void Tree::sethSize(int hSize_, int j) {

    hSize[j] = hSize_;
    root->sethSize(hSize_,j);
  } 

  void Tree::updateKinematics(double t) {
    root->updateKinematics(t);
  }

  void Tree::updateJacobians(double t) {
    root->updateJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Tree::updateSecondJacobians(double t) {
    root->updateSecondJacobians(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (*i)->updateJacobians(t);
  }

  void Tree::updatedu(double t, double dt) {

    ud = slvLLFac(LLM, h*dt+r);
  }

  void Tree::updatezd(double t) {

    qd = T*u;
    ud =  slvLLFac(LLM, h+r);

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatexd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexd(t);

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexd(t);
  }

  void Tree::facLLM() {
    LLM = facLL(M); 
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
