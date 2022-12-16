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

#include "mbsimFlexibleBody/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"
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
  class Contour1sFlexible : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sFlexible(const std::string & name) : Contour1s(name), sOld(-1e12) { }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

       MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;

       fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override { return evalPosition(zeta(0)); }
       fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override { return evalWs(zeta(0)); }
       fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override { return evalWt(zeta(0)); }
       fmatvec::Vec3 evalWu(const fmatvec::Vec2 &zeta) override { return evalWs(zeta); }
       fmatvec::Vec3 evalWv(const fmatvec::Vec2 &zeta) override { return evalWt(zeta); }

       bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]; }

      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override { return findContactPairingFlexible(type0, type1); }

      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      void resetUpToDate() override;
      virtual void updatePositions(double s);

      fmatvec::Vec3 evalPosition(double s) { if(fabs(s-sOld)>MBSim::macheps) updatePositions(s); return WrOP; }
      fmatvec::Vec3 evalWs(double s) { if(fabs(s-sOld)>MBSim::macheps) updatePositions(s); return Ws; }
      fmatvec::Vec3 evalWt(double s) { if(fabs(s-sOld)>MBSim::macheps) updatePositions(s); return Wt; }

      void updatePositions(MBSim::ContourFrame *frame) override;
      void updateVelocities(MBSim::ContourFrame *frame) override;
      void updateAccelerations(MBSim::ContourFrame *frame) override;
      void updateJacobians(MBSim::ContourFrame *frame, int j=0) override;
      void updateGyroscopicAccelerations(MBSim::ContourFrame *frame) override;

    protected:
      fmatvec::Vec3 WrOP, Ws, Wt;

      double sOld;
  };

}

#endif /* _CONTOUR1S_FLEXIBLE_H_ */
