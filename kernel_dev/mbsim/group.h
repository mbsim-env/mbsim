/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _GROUP_H_
#define _GROUP_H_

#include <mbsim/dynamic_system.h>

namespace MBSim {

  /**
   * \brief group ingredients do not depend on each other
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   */
  class Group : public DynamicSystem {
    public:
      /**
       * \brief constructor
       * \param name of group
       */
      Group(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Group();

      /* INHERITED INTERFACE OF SUBSYSTEM */
      virtual void updateJacobians(double t);
      virtual void facLLM();
      using DynamicSystem::addObject;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t);
      virtual void updatedu(double t, double dt);
      virtual void updatezd(double t);
      virtual void updateInverseKineticsJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Group"; }
      /***************************************************/

      /**
       * \param dynamic system to add
       * \param relative position of dynamic system
       * \param relative orientation of dynamic system
       * \param relation frame
       */
      void addDynamicSystem(DynamicSystem *dynamicsystem, const fmatvec::Vec &RrRD, const fmatvec::SqrMat &ARD, const Frame* refFrame=0);
  };
}

#endif

