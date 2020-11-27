/* Copyright (C) 2004-2020 MBSim Development Team
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

#ifndef _CONTACT_KINEMATICS_SPATIALCONTOUR_SPATIALCONTOUR_H_
#define _CONTACT_KINIMATICS_SPATIALCONTOUR_SPATIALCONTOUR_H_

#include "contact_kinematics.h"

namespace MBSim {

  class FuncPairSpatialContourSpatialContour;

  /*! \brief pairing spatial contour to spatial contour
   * \author Martin Foerg
   */
  class ContactKinematicsSpatialContourSpatialContour : public ContactKinematics {
    public:
      ContactKinematicsSpatialContourSpatialContour() = default;
      ~ContactKinematicsSpatialContourSpatialContour();

      /* INHERITED INTERFACE */
      void calcisSize() override { isSize = 4*maxNumContacts; }
      void assignContours(const std::vector<Contour*> &contour) override;
      void setInitialGuess(const fmatvec::MatV &zeta0_) override;
      void search() override;
      void updateg(SingleContact &contact, int i=0) override;
      /***************************************************/

    private:
      /**
       * \brief root function
       */
      MBSim::FuncPairSpatialContourSpatialContour *func;
  };

}

#endif
