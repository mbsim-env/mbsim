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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_KINEMATICS_POINT_NURBS_2S_H_
#define _CONTACT_KINEMATICS_POINT_NURBS_2S_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"
//#include "mbsim/contours/contour2s.h"

namespace MBSim {
  class Point;
  class FuncPairContour2sPoint;
  class Contour2s;
}

namespace MBSimFlexibleBody {

//  class FlexibleBodyLinearExternalFFR;

  /**
   * \brief pairing point to contour2s surface
   * \author: Zhan Wang
   * \date 2013-12-05
   */
  class ContactKinematicsPointContour2s : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointContour2s() {}

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsPointContour2s();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<MBSim::Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData* cpData) { throw MBSim::MBSimError("ERROR (ContactKinematicsPointContour2s::updatewb): Not implemented!"); };
      /***************************************************/

    private:
      /**
       * \brief index for point and contour2s surface
       */
      int ipoint, icontour2s;

      /** 
       * \brief pointer to point and contour2s surface
       */
      MBSim::Point *point;
      MBSim::Contour2s *contour2s;

      /**
       * \brief root function
       */
      MBSim::FuncPairContour2sPoint *func;
  };

}

#endif

