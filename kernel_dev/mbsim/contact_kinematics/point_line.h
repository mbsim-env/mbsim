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

#ifndef _CONTACT_KINEMATICS_POINT_LINE_H_
#define _CONTACT_KINEMATICS_POINT_LINE_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Point;
  class Line;

  /**
   * \brief pairing point to line
   * \author Martin Foerg
   * \date 2009-04-02 some comments (Thorsten Schindler)
   */
  class ContactKinematicsPointLine : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(Vec &g, ContourPointData *cpData);
      virtual void updatewb(Vec &wb, const Vec &g, ContourPointData *cpData);
      /***************************************************/

    private:
      /** 
       * \brief contour index
       */
      int ipoint, iline;
      
      /**
       * \brief contour classes
       */
      Point *point;
      Line *line;

  };

}

#endif

