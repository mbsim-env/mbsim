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

#ifndef _CONTACT_KINEMATICS_CIRCLEPLANARCONTOUR_H_
#define _CONTACT_KINEMATICS_CIRCLEPLANARCONTOUR_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Circle;
  class FuncPairPlanarContourCircle;

  /**
   * \brief pairing outer circle side to planar contour
   * \author Martin Foerg
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   */
  class ContactKinematicsCirclePlanarContour : public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCirclePlanarContour() : icircle(0), iplanarcontour(0), circle(NULL), planarcontour(NULL), func(NULL), searchAllCP(false) { }

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsCirclePlanarContour();

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
      int icircle, iplanarcontour;

      /**
       * \brief contour classes
       */
      Circle *circle;
      Contour *planarcontour;

      /**
       * \brief root function
       */
      FuncPairPlanarContourCircle *func;

      bool searchAllCP;
  };

}

#endif

