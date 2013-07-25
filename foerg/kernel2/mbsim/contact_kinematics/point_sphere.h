/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTACT_KINEMATICS_POINT_SPHERE_H_
#define _CONTACT_KINEMATICS_POINT_SPHERE_H_

#include "contact_kinematics.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {

  class Point;
  class Sphere;

  /**
   * \brief pairing point to sphere
   * \author Martin Foerg
   * \todo change stage to new interface TODO
   */
  class ContactKinematicsPointSphere : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData *cpData) { throw MBSimError("ERROR (ContactKinematicsPointSphere:updatewb): Not implemented!"); }
      //virtual void stage1(fmatvec::Vec &g, std::vector<ContourPointData> &cpData);
      //virtual void stage2(const fmatvec::Vec &g, fmatvec::Vec &gd, std::vector<ContourPointData> &cpData);
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int ipoint, isphere;
      
      /**
       * \brief contour classes
       */
      Point *point;
      Sphere *sphere;

  };

}

#endif

