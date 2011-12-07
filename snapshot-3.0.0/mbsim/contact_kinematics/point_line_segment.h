/* Copyright (C) 2004-2011 MBSim Development Team
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

#ifndef _CONTACT_KINEMATICS_POINT_LINE_SEGMENT_H_
#define _CONTACT_KINEMATICS_POINT_LINE_SEGMENT_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Point;
  class LineSegment;

  /**
   * \brief pairing point to line segment
   * \author Martin Foerg
   * \date 2010-05-20 initial commit (Martin Foerg)
   * \date 2011-01-27 some comments (Thomas Cebulla)
   */
  class ContactKinematicsPointLineSegment : public ContactKinematics {
    public:
      /* INHERITED INTERFACE OF CONTACT_KINEMATICS */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData *cpData);
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
      LineSegment *line;

  };

}

#endif

