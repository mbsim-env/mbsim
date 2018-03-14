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
      ContactKinematicsPointExtrusion()  { }
      
      /**
       * \brief destructor
       */
      ~ContactKinematicsPointExtrusion() override;

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<MBSim::Contour*>& contour) override;
      void updateg(SingleContact &contact, int i=0) override;
      /***************************************************/

      void setSearchAllContactPoints(bool searchAllCP_=true) override { searchAllCP = searchAllCP_; }

    protected:
      /** 
       * \brief contour index 
       */
      int ipoint{0};
      int iextrusion{0};

      /** 
       * \brief contour classes 
       */
      MBSim::Point *point{0};
      Contour *extrusion{0};

      MBSim::FuncPairPlanarContourPoint *func;

      bool searchAllCP{false};
  };

}

#endif
