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

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsim/utils/eps.h"

namespace MBSimFlexibleBody {

  /**
   * \brief flexible band contour for spatial curves
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-10 calculation of Jacobian of Translation for Contours (Thorsten Schindler)
   */
  class FlexibleBand : public Contour1sFlexible {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FlexibleBand(const std::string& name) : Contour1sFlexible(name), width(0), ARK(fmatvec::EYE) { }

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBand"; }
     /***************************************************/

      /* GETTER / SETTER */
      void setWidth(double width_) { width = width_; }
      double getWidth() const { return width; }

      void setRelativePosition(const fmatvec::Vec2 &r);
      void setRelativeOrientation(double al);

      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARK; }

      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta) { return getPosition(t,zeta(0)); }
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta(0)); }

      virtual bool isZetaOutside(const fmatvec::Vec2 &zeta) { return zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1] or zeta(1) < -0.5*width or zeta(1) > 0.5*width; }

      void updatePositions(double t, double s);

      fmatvec::Vec3 getPosition(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return WrOP; }
      fmatvec::Vec3 getWt(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return Wt; }

    protected:
      /**
       * \brief width of flexible band
       */
      double width;

      fmatvec::Vec3 RrRP, WrOP, Wt;
      fmatvec::SqrMat3 ARK;
  };

}

#endif /* _FLEXIBLE_BAND_H_ */
