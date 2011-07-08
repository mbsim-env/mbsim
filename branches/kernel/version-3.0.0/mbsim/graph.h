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


#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "mbsim/dynamic_system.h"

namespace H5 {
  class Group;
}

namespace MBSim {


  /**
   * \brief class for tree-structured mechanical systems with recursive and flat memory mechanism
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Graph : public DynamicSystem {
    public:
      /**
       * \brief constructor
       * \param name of tree
       */
      Graph(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Graph();

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t);
      virtual void updatedu(double t, double dt);
      virtual void updatezd(double t);
      virtual void updateud(double t, int i=0);
      virtual void sethSize(int h, int j=0);
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      //virtual void updateInverseKineticsJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Graph"; }
      /***************************************************/

      /* INHERITED INTERFACE OF SUBSYSTEM */
      virtual void updateJacobians(double t, int j=0);
      void facLLM(int i=0); 
      /***************************************************/

      /**
       * \brief add new object to graph at level
       * \param level
       * \param object
       */
      void addObject(int level, Object* object); 

    protected:
      /**
       * \brief none
       */
      std::vector< std::vector<Object*> > obj;
  };

}

#endif

