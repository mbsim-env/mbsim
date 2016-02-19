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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTACT_KINEMATICS_POINT_PLANAR_CONTOUR_H_
#define _CONTACT_KINEMATICS_POINT_PLANAR_CONTOUR_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Point;
  class FuncPairPlanarContourPoint;

  /**
   * \brief pairing point to planar contour
   * \author Roland Zander
   * \date 2009-04-02 some comments (Thorsten Schindler)
   * \date 2009-06-04 new interface (Thorsten Schindler)
   */
  class ContactKinematicsPointPlanarContour: public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointPlanarContour() : ContactKinematics(), ipoint(0), icontour(0), point(0), contour1s(0), searchAllCP(false) { }

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsPointPlanarContour();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(double t, double &g, std::vector<ContourFrame*> &cFrame, int index = 0);
      virtual void updatewb(double t, fmatvec::Vec &wb, double g, std::vector<ContourFrame*> &cFrame);
      /***************************************************/

      void setSearchAllContactPoints(bool searchAllCP_=true) { searchAllCP = searchAllCP_; }

   private:
      /**
       * \brief contour index
       */
      int ipoint, icontour;
      
      /**
       * \brief contour classes
       */
      Point *point;
      Contour *contour1s;

      /**
       * \brief root function
       */
      FuncPairPlanarContourPoint *func;

      bool searchAllCP;
  };

}

#endif
