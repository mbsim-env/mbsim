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

#ifndef _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_
#define _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {
  class CircleHollow;
  class FuncPairContour1sCircleHollow;
}

namespace MBSimFlexibleBody {

  class CylinderFlexible;

  /**
   * \brief pairing CircleHollow to CylinderFlexible
   * \author Roland Zander
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   * \todo change stage to new interface TODO
   */
  class ContactKinematicsCircleHollowCylinderFlexible : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCircleHollowCylinderFlexible() {}

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsCircleHollowCylinderFlexible();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<MBSim::Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData* cpData) { throw MBSim::MBSimError("ERROR (ContactKinematicsCircleHollowCylinderFlexible::updatewb): Not implemented!"); };
      /***************************************************/
    
    private:
      /**
       * \brief contour index
       */
      int icircle, icylinder;

      /**
       * \brief contour classes
       */
      MBSim::CircleHollow *circle;
      CylinderFlexible *cylinder;

      /**
       * \brief root function
       */
      MBSim::FuncPairContour1sCircleHollow *func;
  };

}

#endif /* _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_ */

