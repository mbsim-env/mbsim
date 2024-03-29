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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_KINEMATICS_POINT_CYLINDERFLEXIBLE_H_
#define _CONTACT_KINEMATICS_POINT_CYLINDERFLEXIBLE_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {
  class Point;
  class FuncPairContour1sPoint;
}

namespace MBSimFlexibleBody {

  class CylinderFlexible;

  /**
   * \brief pairing point to cylinder flexible
   * \author: Roland Zander
   * \date 2009-03-19 changes for new kernel (Thorsten Schindler)
   */
  class ContactKinematicsPointCylinderFlexible : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointCylinderFlexible() = default;

      /**
       * \brief destructor
       */
       ~ContactKinematicsPointCylinderFlexible() override;

      /* INHERITED INTERFACE */
       void assignContours(const std::vector<MBSim::Contour*> &contour) override;
      virtual void updateg(double &g, std::vector<MBSim::Frame*> &cFrame, int index = 0);
      /***************************************************/

    private:
      /**
       * \brief index for point and cylinder
       */
      int ipoint, icylinder;

      /** 
       * \brief pointer to point and cylinder
       */
      MBSim::Point *point;
      CylinderFlexible *cylinder;

      /**
       * \brief root function
       */
      MBSim::FuncPairContour1sPoint *func;

      /*!
       * \brief flag to check if local search can be used
       */
      bool useLocal { false };
  };

}

#endif

