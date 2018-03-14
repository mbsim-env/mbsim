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
 */

#ifndef CIRCLESOLID_FLEXIBLEBAND_H_
#define CIRCLESOLID_FLEXIBLEBAND_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSim {
  class Circle;
  class Contour;
}

namespace MBSimFlexibleBody {

  /**
   Contour \brief pairing solid cirlce to flexible band, planar only
   * \author Roland Zander
   * \date 2009-03-07 initial commit (Roland Zander)
   */
  class ContactKinematicsCircleFlexibleBand : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCircleFlexibleBand();

      /*!
      */
      void setNumberOfPossibleContactPerNode(int n) { possibleContactsPerNode = n; }

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<MBSim::Contour*>& contour) override;
      /***************************************************/

    private:
      /** 
       * \brief contour index 
       */
      int icircle{0}, icontour{0};

      /**
       * \brief possible contacts regarded per node
       */
      int possibleContactsPerNode{1};

      /** 
       * \brief contour classes 
       */
      MBSim::Circle *circle{0};
      MBSim::Contour *extrusion{0};

      fmatvec::Vec staticNodes;

      std::vector<ContactKinematics*> contactKinematics;
  };

}

#endif
