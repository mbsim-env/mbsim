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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _OBJECT_HYDRAULICS_H_
#define  _OBJECT_HYDRAULICS_H_

#include "mbsim/object.h"

namespace MBSim {
  class ObjectHydraulics : public Object {
    public:
      ObjectHydraulics(const std::string &name);
      ~ObjectHydraulics() {}
      virtual std::string getType() const { return "ObjectHydraulics"; }

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateStateDependentVariables(double t) {};
      virtual void updateJacobians(double t) {};
      virtual void updateInverseKineticsJacobians(double t) {};
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
      /***************************************************/

  };
}

#endif   /* ----- #ifndef _OBJECT_HYDRAULICS_H_  ----- */

