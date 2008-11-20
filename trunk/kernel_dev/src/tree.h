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

#ifndef _TREE_H_
#define _TREE_H_

#include "subsystem.h"


using namespace std;


namespace MBSim {

  class Tree;

  class Node {
    protected:
      vector<Node*> child;
      ObjectInterface *obj;
    public:
      Node(ObjectInterface* obj_) : obj(obj_) {}
      ~Node() {}
      void addChild(Node* child); 
      void updateKinematics(double t);
      void calcqSize(int &size);
      void calcuSize(int &size);
      void sethSize(int &size);
  };

  /*! \brief class for tree-structured systems with only rigid bodies
   *
   * */
  class Tree : public Subsystem {

    protected:
      Node* root;

    public:

    Tree(const string &projectName);
    ~Tree();

    void calcqSize();
    void calcuSize();
    void sethSize(int h);

    void facLLM(); 
    void updatezd(double t);
    void updatedu(double t, double dt);
    void updateKinematics(double t);

    double computePotentialEnergy();

    Node* addObject(Node* node, Object* obj);
    Node* addSubsystem(Node* node, Subsystem* sys, const Vec &RrRS, const SqrMat &ARS, const CoordinateSystem* refCoordinateSystem=0);

  };

}

#endif
