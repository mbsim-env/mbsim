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
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_KINEMATICS_POINT_CONTOUR1S_H_
#define _CONTACT_KINEMATICS_POINT_CONTOUR1S_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Point;
  class Contour1s;

  /**
   * \brief pairing point to Contour1s
   * \author Roland Zander
   * \date 2009-04-02 some comments (Thorsten Schindler)
   * \date 2009-06-04 new interface (Thorsten Schindler)
   */
  class ContactKinematicsPointContour1s: public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointContour1s();

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsPointContour1s();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData *cpData) { throw MBSimError("ERROR (ContactKinematicsPointContour1s::updatewb): Not implemented!"); }
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int ipoint, icontour;
      
      /**
       * \brief contour classes
       */
      Point *point;
      Contour1s *contour1s;
  };

}

#endif /* _CONTACT_KINEMATICS_POINT_CONTOUR1S_H_ */

