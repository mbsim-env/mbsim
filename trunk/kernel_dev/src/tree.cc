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
#include "extra_dynamic_interface.h"

namespace MBSim {

  Tree::Tree(const string &projectName) : Subsystem(projectName) {
  }

  Tree::~Tree() {
  }

  void Tree::calchSize() {

    for(unsigned i=0; i<subsystem.size(); i++) {
      int j = subsystem.size()-1-i;
      if(i==0)
        subsystem[j]->sethSize(hSize);
      else
        subsystem[j]->sethSize(subsystem[j+1]->gethSize() - subsystem[j+1]->getuSize());
      subsystem[j]->calchSize();
    }

    for(unsigned i=0; i<object.size(); i++) {
      int j = object.size()-1-i;
      if(i==0)
        object[j]->sethSize(hSize);
      else
        object[j]->sethSize(object[j+1]->gethSize() - object[j+1]->getuSize());
      object[j]->calchSize();
    }
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
    // FACLLM computes Cholesky decomposition of the mass matrix
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
