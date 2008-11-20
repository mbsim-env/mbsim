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

  class TreeElement {
    protected:
      vector<TreeElement*> child;
      ObjectInterface *obj;
    public:
      TreeElement(ObjectInterface* obj_) : obj(obj_) {}
      ~TreeElement() {}
      void addChild(TreeElement* child); 
      void updateKinematics(double t);
      void calcqSize(int &s);
      void calcuSize(int &s);
   int sethSize(int hSize);
  //  void sethSize(int &hSize);
  };

  /*! \brief class for tree-structured systems with only rigid bodies
   *
   * */
  class Tree : public Subsystem {

    protected:
      TreeElement* root;

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

    TreeElement* addObject(TreeElement* node, Object* obj);
    TreeElement* addSubsystem(TreeElement* node, Subsystem* sys, const Vec &RrRS, const SqrMat &ARS, const CoordinateSystem* refCoordinateSystem=0);

  };

}

#endif
