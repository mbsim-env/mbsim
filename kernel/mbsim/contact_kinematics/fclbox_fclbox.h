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

#ifndef _CONTACT_KINEMATICS_FCLBOX_FCLBOX_H_
#define _CONTACT_KINEMATICS_FCLBOX_FCLBOX_H_

#include "contact_kinematics.h"
#include "fcl/narrowphase/collision_object.h"

namespace MBSim {

  class FCLBox;

  /** 
   * \brief pairing box to box using FCL
   * \author Martin Foerg
   */
  class ContactKinematicsFCLBoxFCLBox : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(double &g, std::vector<ContourFrame*> &cFrame, int index = 0) override;
      void updatewb(fmatvec::Vec &wb, double g, std::vector<ContourFrame*> &cFrame) override { throw std::runtime_error("(ContactKinematicsFCLBoxFCLBox::updatewb): Not implemented!"); };
      /***************************************************/

    protected:
      /**
       * \brief contour index
       */
      int ibox0, ibox1;

      /**
       * \brief contour classes
       */
      FCLBox *box0, *box1;

      std::shared_ptr<fcl::CollisionObject<double> > obj0, obj1;
  };

}

#endif
