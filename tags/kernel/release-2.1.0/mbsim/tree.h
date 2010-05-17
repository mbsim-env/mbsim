/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */


#ifndef _TREE_H_
#define _TREE_H_

#include "mbsim/dynamic_system.h"

namespace H5 {
  class Group;
}

namespace MBSim {

  /**
   * \brief recursive standard tree structure
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Node {
    public:
      /**
       * \brief constructor
       * \param object interface
       */
      Node(ObjectInterface* obj_) : obj(obj_) {}

      /**
       * \brief destructor
       */
      virtual ~Node() {}

      /**
       * \param child node to add
       */
      void addChild(Node* child); 

      /**
       * \brief recursive kinematics update
       * \param time
       */
      void updateStateDependentVariables(double t);

      /**
       * \brief recursive JACOBIAN for acceleration description update
       * \param time
       */
      void updateJacobians(double t);

      /**
       * \brief recursive JACOBIAN for inverse kinetics update
       * \param time
       */
      void updateInverseKineticsJacobians(double t);

      /**
       * \brief recursive calculation of position size
       * \param position size
       */
      void calcqSize(int &qsize);

      /**
       * \brief recursive calculation of velocity size
       * \param velocity size
       * \param index for normal usage and inverse kinetics
       */
      void calcuSize(int &usize, int j=0);

      /**
       * \brief recursive setting of smooth right hand side size
       * \param smooth right hand side size
       * \param index for normal usage and inverse kinetics
       */
      void sethSize(int &hsize, int j=0);

      ObjectInterface* getObject() {return obj;} 

    protected:
      /**
       * \brief vector of node pointer for recursive structure
       */
      std::vector<Node*> child;

      /**
       * \brief pointer of object interface
       */
      ObjectInterface *obj;
  };

  /**
   * \brief class for tree-structured mechanical systems with recursive and flat memory mechanism
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Tree : public DynamicSystem {
    public:
      /**
       * \brief constructor
       * \param name of tree
       */
      Tree(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Tree();

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t);
      virtual void updatedu(double t, double dt);
      virtual void updatezd(double t);
      virtual void sethSize(int h, int j=0);
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      virtual void updateInverseKineticsJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Tree"; }
      /***************************************************/

      /* INHERITED INTERFACE OF SUBSYSTEM */
      virtual void updateJacobians(double t);
      void facLLM(); 
      /***************************************************/

      /**
       * \brief add new object to tree
       * \param tree
       * \param object
       * \return new node of object
       */
      Node* addObject(Node* tree, Object* obj);

      /**
       * \brief add new tree to tree
       * \param tree
       * \param dynamic system
       * \return new node of dynamic system
       */
      Node* addTree(Node* tree, Tree* sys);

    protected:
      /**
       * \brief root node of tree
       */
      Node* root;
  };

}

#endif

