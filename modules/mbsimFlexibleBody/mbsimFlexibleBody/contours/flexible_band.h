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

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsim/contour.h"

#include "mbsimFlexibleBody/contours/contour_1s_neutral_factory.h"

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
      FlexibleBand(const std::string& name);

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        return "FlexibleBand";
      }
      /***************************************************/

      virtual MBSim::Frame* createContourFrame(const std::string &name="P");

      /* INHERITED INTERFACE OF CONTOUR */
//      virtual void updateKinematicsForFrame(MBSim::ContourPointData& cp, MBSim::Frame::Frame::Feature ff)
//      virtual void updateJacobiansForFrame(const fmatvec::Vec2 &zeta, int j = 0);
      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getVelocity(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getAngularVelocity(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Mat3xV getJacobianOfTranslation(double t, const fmatvec::Vec2 &zeta, int j=0);
      virtual fmatvec::Mat3xV getJacobianOfRotation(double t, const fmatvec::Vec2 &zeta, int j=0);
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta) { return getWu(t,zeta); }
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta);
//      virtual fmatvec::Vec3 getVelocity(double t, const fmatvec::Vec2 &zeta);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
//      virtual void computeRootFunctionPosition(const fmatvec::Vec2 &zeta) { Contour1sFlexible::updateKinematicsForFrame(cp, MBSim::Frame::position); }
//      virtual void computeRootFunctionFirstTangent(const fmatvec::Vec2 &zeta) { Contour1sFlexible::updateKinematicsForFrame(cp, MBSim::Frame::firstTangent); }
      /***************************************************/

      /* GETTER / SETTER */
      void setCn(const fmatvec::Vec& Cn_);
      void setWidth(double width_);

      /*!
       * \brief set normal distance of band surface to fibre of reference of one dimensional continuum
       * \param normal distance
       */
      void setNormalDistance(double nDist_);

      /*!
       * \brief get normal distance of band surface to fibre of reference of one dimensional continuum
       * \return normal distance
       */
      double getNormalDistance() {
        return nDist;
      }
      double getWidth() const;
      /***************************************************/

    protected:
      /**
       * \brief normal of flexible band with respect to referencing neutral fibre (outward normal = (n b)*Cn)
       */
      fmatvec::Vec Cn;

      /** 
       * \brief width of flexible band
       */
      double width;

      /**
       * \brief distance from the referencing neutral fibre in direction of given normal
       */
      double nDist;

  };

  inline void FlexibleBand::setWidth(double width_) {
    width = width_;
  }
  inline void FlexibleBand::setNormalDistance(double nDist_) {
    nDist = nDist_;
  }
  inline double FlexibleBand::getWidth() const {
    return width;
  }

}

#endif /* _FLEXIBLE_BAND_H_ */

