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

#ifndef _CONTOUR1S_FLEXIBLE_H_
#define _CONTOUR1S_FLEXIBLE_H_

#include "mbsim/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "contour_1s_neutral_factory.h"

namespace MBSim {
  class ContactKinematics;
}

namespace MBSimFlexibleBody {

  /** 
   * \brief numerical description of contours with one contour parameter
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-03-18 initial comment (Thorsten Schindler)
   * \date 2009-04-05 adapted to non-template FlexibleBody (Schindler / Zander)
   * \date 2009-06-04 new file (Thorsten Schindler)
   *
   * \todo: make this class to be the neutral factory...
   *        For it all "natural" contours of the bodies would have to implement a neutral_contour
   *        Then it would not be the case, that the neutral_contour1s as a contour1s as well as this contour1s_flexible
   */
  class Contour1sFlexible : public MBSim::Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sFlexible(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        return "Contour1sFlexible";
      }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff) {
        if (neutral)
          neutral->updateKinematicsForFrame(cp, ff);
        else
          static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp, ff); //TODO: avoid asking parent body here!
      }
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0) {
        if (neutral)
          neutral->updateJacobiansForFrame(cp);
        else
          static_cast<FlexibleBody*>(parent)->updateJacobiansForFrame(cp);
      }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) {
        if (neutral)
          neutral->updateKinematicsForFrame(cp, MBSim::position);
        else
          updateKinematicsForFrame(cp, MBSim::position);
      }
      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) {
        if (neutral)
          neutral->updateKinematicsForFrame(cp, MBSim::firstTangent);
        else
          updateKinematicsForFrame(cp, MBSim::firstTangent);
      }
      virtual void computeRootFunctionNormal(MBSim::ContourPointData &cp) {
        if (neutral)
          neutral->updateKinematicsForFrame(cp, MBSim::normal);
        else
          updateKinematicsForFrame(cp, MBSim::normal);
      }
      virtual void computeRootFunctionSecondTangent(MBSim::ContourPointData &cp) {
        if (neutral)
          neutral->updateKinematicsForFrame(cp, MBSim::secondTangent);
        else
          updateKinematicsForFrame(cp, MBSim::secondTangent);
      }
      /***************************************************/

      MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) {
        return findContactPairingFlexible(type0.c_str(), type1.c_str());
      }

      void setNeutral(Contour1sNeutralFactory* neutral_) {
        neutral = neutral_;
      }

    protected:
      /*!
       * \brief object for 1s-flexible curves that is the interface
       *
       * \todo: maybe this actually should be used for all 1s contours (as the same interface?)
       */
      Contour1sNeutralFactory* neutral;

  };

}

#endif /* _CONTOUR1S_FLEXIBLE_H_ */

