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

#ifndef CIRCLESOLID_FLEXIBLEBAND_H_
#define CIRCLESOLID_FLEXIBLEBAND_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {
  class CircleSolid;
}

namespace MBSimFlexibleBody {

  class FlexibleBand;

  /**
   * \brief pairing solid cirlce to flexible band, planar only
   * \author Roland Zander
   * \date 2009-03-07 initial commit (Roland Zander)
   */
  class ContactKinematicsCircleSolidFlexibleBand : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCircleSolidFlexibleBand();

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsCircleSolidFlexibleBand();

      /*!
      */
      void setNumberOfPossibleContactPerNode(int n) {possibleContactsPerNode = n;}

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      virtual void assignContours(const std::vector<MBSim::Contour*>& contour);
      //      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData);   
      virtual void updateg(std::vector<fmatvec::Vec>::iterator ig, std::vector<MBSim::ContourPointData*>::iterator icpData);   
      //      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g,ContourPointData *cpData) { throw MBSimError("ERROR (ContactKinematicsCircleSolidFlexibleBand::updatewb): not implemented!"); }   
      virtual void updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData) { throw MBSim::MBSimError("ERROR (ContactKinematicsCircleSolidFlexibleBand::updateg): Not implemented!"); }
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData* cpData) { throw MBSim::MBSimError("ERROR (ContactKinematicsCircleSolidFlexibleBand::updatewb): Not implemented!"); };
      /***************************************************/

    private:
      /** 
       * \brief contour index 
       */
      int icircle, icontour;

      /**
       * \brief possible contacts regarded per node
       */
      int possibleContactsPerNode;

      /**
       * \brief radius of circle
       */
      double rCircle;

      /**
       * \brief half thickness of band
       */
      double hBand;

      /**
       * \brief width of band
       */
      double wBand;

      /** 
       * \brief contour classes 
       */
      MBSim::CircleSolid *circle;
      FlexibleBand *band;

      fmatvec::Vec lastContactPositions;
      fmatvec::Vec staticNodes;
      double epsTol;
      double l0;
  };

}

#endif /* CIRCLESOLID_FLEXIBLEBAND_H_ */

