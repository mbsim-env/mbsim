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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTOUR1S_FLEXIBLE_H_
#define _CONTOUR1S_FLEXIBLE_H_

#include "mbsim/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "contour_1s_neutral_factory.h"
#include "mbsim/utils/eps.h"

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
      Contour1sFlexible(const std::string & name) : Contour1s(name), neutral(0), sOld(-1e12) { }

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contour1sFlexible"; }
      /***************************************************/

      virtual void init(InitStage stage);

      virtual MBSim::ContourFrame* createContourFrame(const std::string &name="P");

      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta(0)); }
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta); }
      virtual fmatvec::Vec3 getWv(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta); }

      MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) { return findContactPairingFlexible(type0.c_str(), type1.c_str()); }

      void setNeutral(Contour1sNeutralFactory* neutral_) { neutral = neutral_; }

      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      void resetUpToDate() { sOld = -1e12; }
      virtual void updatePositions(double t, double s);

      fmatvec::Vec3 getWs(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return Ws; }

    protected:
      /*!
       * \brief object for 1s-flexible curves that is the interface
       *
       * \todo: maybe this actually should be used for all 1s contours (as the same interface?)
       */
      Contour1sNeutralFactory* neutral;

      fmatvec::Vec3 Ws;

      Frame1s P;

      double sOld;
  };

}

#endif /* _CONTOUR1S_FLEXIBLE_H_ */
