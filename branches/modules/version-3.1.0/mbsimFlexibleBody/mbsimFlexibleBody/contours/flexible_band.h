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

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsim/contour.h"

namespace MBSimFlexibleBody {

  /**
   * \brief flexible band contour for spatial curves
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-07-10 calculation of Jacobian of Translation for Contours (Thorsten Schindler)
   */
  template <class Col>
  class FlexibleBand : public Contour1sFlexible<Col> {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FlexibleBand(const std::string& name);

      /*!
       * \brief destructor
       */
      virtual ~FlexibleBand() {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        fmatvec::Vector<Col, double>* vec = new fmatvec::Vector<Col, double>;
        std::string appendix = fmatvec::getVectorTemplateType(vec);
        delete vec;
        return "FlexibleBand" + appendix;
      }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(MBSim::ContourPointData& cp, MBSim::FrameFeature ff);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual fmatvec::Vec3 computePosition(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::position_cosy);
        return cp.getFrameOfReference().getPosition();
      }
      virtual fmatvec::Vec3 computeVelocity(MBSim::ContourPointData &cp) {
        updateKinematicsForFrame(cp, MBSim::velocity_cosy);
        return cp.getFrameOfReference().getVelocity();
      }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(MBSim::ContourPointData &cp) {
        Contour1sFlexible<Col>::updateKinematicsForFrame(cp, MBSim::position);
      }
      virtual void computeRootFunctionFirstTangent(MBSim::ContourPointData &cp) {
        Contour1sFlexible<Col>::updateKinematicsForFrame(cp, MBSim::firstTangent);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setCn(const fmatvec::Vec2& Cn_);
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
      ;
      double getWidth() const;
      /***************************************************/

    private:
      /**
       * \brief normal of flexible band with respect to referencing neutral fibre (outward normal = (n b)*Cn)
       */
      fmatvec::Vec2 Cn;

      /** 
       * \brief width of flexible band
       */
      double width;

      /**
       * \brief distance from the referencing neutral fibre in direction of given normal
       */
      double nDist;
  };

  template <class Col>
  inline void FlexibleBand<Col>::setWidth(double width_) {
    width = width_;
  }
  template <class Col>
  inline void FlexibleBand<Col>::setNormalDistance(double nDist_) {
    nDist = nDist_;
  }
  template <class Col>
  inline double FlexibleBand<Col>::getWidth() const {
    return width;
  }

  template <class Col>
  FlexibleBand<Col>::FlexibleBand(const std::string& name) :
      Contour1sFlexible<Col>(name), Cn(), width(0.), nDist(0.) {
  }

  template <class Col>
  inline void FlexibleBand<Col>::setCn(const fmatvec::Vec2& Cn_) {
    //assert(Cn_.size() == 2);
    Cn = Cn_ / nrm2(Cn_);
  }

  template <class Col>
  inline void FlexibleBand<Col>::updateKinematicsForFrame(MBSim::ContourPointData& cp, MBSim::FrameFeature ff) {
    if (ff == MBSim::firstTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy)
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::firstTangent);
    if (ff == MBSim::normal || ff == MBSim::secondTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy) {
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::normal);
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::secondTangent);
      fmatvec::Vec3 WnLocal = cp.getFrameOfReference().getOrientation().col(0);
      fmatvec::Vec3 WbLocal = cp.getFrameOfReference().getOrientation().col(2);
      if (ff != MBSim::secondTangent)
        cp.getFrameOfReference().getOrientation().set(0, WnLocal * Cn(0) + WbLocal * Cn(1));
      if (ff != MBSim::normal)
        cp.getFrameOfReference().getOrientation().set(2, -WnLocal * Cn(1) + WbLocal * Cn(0));
    }
    if (ff == MBSim::position || ff == MBSim::position_cosy) {
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::position);
      cp.getFrameOfReference().getPosition() += cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
    }
    if (ff == MBSim::angularVelocity || ff == MBSim::velocities || ff == MBSim::velocities_cosy) {
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::angularVelocity);
    }
    if (ff == MBSim::velocity || ff == MBSim::velocity_cosy || ff == MBSim::velocities || ff == MBSim::velocities_cosy) {
      static_cast<FlexibleBody<Col>*>(this->parent)->updateKinematicsForFrame(cp, MBSim::velocity);
      fmatvec::Vec3 dist = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
      cp.getFrameOfReference().getVelocity() += crossProduct(cp.getFrameOfReference().getAngularVelocity(), dist);
    }
  }

  template <class Col>
  inline void FlexibleBand<Col>::updateJacobiansForFrame(MBSim::ContourPointData &cp, int j /*=0*/) {
    static_cast<FlexibleBody<Col>*>(this->parent)->updateJacobiansForFrame(cp);
    fmatvec::Vec3 WrPC = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1); // vector from neutral line to contour surface point
    fmatvec::SqrMat3 tWrPC = tilde(WrPC); // tilde matrix of above vector
    cp.getFrameOfReference().setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation() - tWrPC * cp.getFrameOfReference().getJacobianOfRotation()); // Jacobian of translation at contour surface with standard description assuming rigid cross-section
  }

}

#endif /* _FLEXIBLE_BAND_H_ */

