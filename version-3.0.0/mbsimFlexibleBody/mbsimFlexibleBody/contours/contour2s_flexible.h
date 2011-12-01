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
 */

#ifndef _CONTOUR2SFLEXIBLE_H_
#define _CONTOUR2SFLEXIBLE_H_

#include "mbsim/contours/contour_2s.h"

namespace MBSim {
  class ContactKinematics;
}

namespace MBSimFlexibleBody {

  /** 
   * \brief numerical description of contours with two contour parameter
   * \author Thorsten Schindler
   * \date 2009-04-21 initial comment (Thorsten Schindler)
   */
  class Contour2sFlexible : public MBSim::Contour2s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour2sFlexible(const std::string &name) : Contour2s(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contour2sFlexible"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff) { static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,ff); }
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp) { static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp); }
      /***************************************************/

      MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) {
        return findContactPairingFlexible(type0.c_str(), type1.c_str());
      }

  };

}

#endif /* _CONTOUR2SFLEXIBLE_H_ */

