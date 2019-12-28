/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _MBSIMFCL_CONTACT_KINEMATICS_FCLCONTOUR_FCLCONTOUR_H_
#define _MBSIMFCL_CONTACT_KINEMATICS_FCLCONTOUR_FCLCONTOUR_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "fcl/narrowphase/collision_object.h"

namespace MBSimFcl {

  class FclContour;

  /** 
   * \brief pairing contour to contour using FCL
   * \author Martin Foerg
   */
  class ContactKinematicsContourContour : public MBSim::ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      ContactKinematicsContourContour(int maxNumContacts=1) : MBSim::ContactKinematics(maxNumContacts) { }
      void assignContours(const std::vector<MBSim::Contour*> &contour) override;
      void updateg(std::vector<MBSim::SingleContact> &contact) override;
      /***************************************************/

    protected:
      /**
       * \brief contour index
       */
      int icontour0, icontour1;

      /**
       * \brief contour classes
       */
      FclContour *contour0, *contour1;

      std::shared_ptr<fcl::CollisionObject<double>> obj0, obj1;
  };

}

#endif
