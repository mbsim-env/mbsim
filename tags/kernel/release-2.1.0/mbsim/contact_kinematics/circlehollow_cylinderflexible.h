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
 * Contact: rzander@users.berlios.de
 */

#ifndef _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_
#define _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_

#include "contact_kinematics.h"

namespace MBSim {

  class CircleHollow;
  class CylinderFlexible;
  class FuncPairContour1sCircleHollow;

  /**
   * \brief pairing CircleHollow to CylinderFlexible
   * \author Roland Zander
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   * \todo change stage to new interface TODO
   */
  class ContactKinematicsCircleHollowCylinderFlexible : public ContactKinematics {
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
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData* cpData) { throw new MBSimError("ERROR (ContactKinematicsCircleHollowCylinderFlexible::updatewb): Not implemented!"); };
      /***************************************************/
    
    private:
      /**
       * \brief contour index
       */
      int icircle, icylinder;

      /**
       * \brief contour classes
       */
      CircleHollow *circle;
      CylinderFlexible *cylinder;

      /**
       * \brief root function
       */
      FuncPairContour1sCircleHollow *func;
  };

}

#endif /* _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_ */

