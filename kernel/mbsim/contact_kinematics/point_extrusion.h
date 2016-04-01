/* Copyright (C) 2004-2015 MBSim Development Team
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

#ifndef POINT_EXTRUSION_H_
#define POINT_EXTRUSION_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {

  class Point;
  class FuncPairPlanarContourPoint;

  /**
   * \brief pairing point to extrusion
   * \author Martin Foerg
   */
  class ContactKinematicsPointExtrusion : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointExtrusion() : ContactKinematics(), ipoint(0), iextrusion(0), point(0), extrusion(0), searchAllCP(false) { }
      
      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsPointExtrusion();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      virtual void assignContours(const std::vector<MBSim::Contour*>& contour);
      virtual void updateg(double &g, std::vector<MBSim::ContourFrame*> &cFrame, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, double g, std::vector<MBSim::ContourFrame*> &cFrame);
      /***************************************************/

      void setSearchAllContactPoints(bool searchAllCP_=true) { searchAllCP = searchAllCP_; }

    protected:
      /** 
       * \brief contour index 
       */
      int ipoint, iextrusion;

      /** 
       * \brief contour classes 
       */
      MBSim::Point *point;
      Contour *extrusion;

      MBSim::FuncPairPlanarContourPoint *func;

      bool searchAllCP;
  };

}

#endif
