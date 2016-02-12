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

      virtual MBSim::Frame* createContourFrame(const std::string &name="P");

      virtual void updatePositions(double t, ContourFrame* frame);
      virtual void updateVelocities(double t, ContourFrame* frame);
      virtual void updateAccelerations(double t, ContourFrame* frame);
      virtual void updateJacobians(double t, ContourFrame* frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, ContourFrame* frame);
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta) { return getWu(t,zeta); }
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta);

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

