/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _CONTACT_KINEMATICS_NODES_CYLINDER_H_
#define _CONTACT_KINEMATICS_NODES_CYLINDER_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {
  class Cylinder;
  class Contour;
}

namespace MBSimFlexibleBody {

  class NodesContour;

  /** 
   * \brief pairing nodes to cylinder
   * \author Martin Foerg
   */
  class ContactKinematicsNodesCylinder : public MBSim::ContactKinematics {
    public:
      void assignContours(const std::vector<MBSim::Contour*> &contour) override;
      void updateg(MBSim::SingleContact &contact, int i=0) override;
      void updatewb(MBSim::SingleContact &contact, int i=0) override;

    protected:
      /**
       * \brief contour index
       */
      int inodes, icylinder;

      /**
       * \brief contour classes
       */
      NodesContour *nodes;
      MBSim::Cylinder *cylinder;
  };

}

#endif
