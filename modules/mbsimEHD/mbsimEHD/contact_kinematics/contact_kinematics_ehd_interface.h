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

#ifndef _CONTACT_KINEMATICS_EHD_INTERFACE_H_
#define _CONTACT_KINEMATICS_EHD_INTERFACE_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"

namespace MBSimEHD {

  /*!
   * \brief interface class for all contact kinematics for EHD computation
   *
   * Remark: The reference system for the computation of all directions is the hollow-coordinate system.
   * For the frustum contour the "height"-direction is the y-direction, whereas the x/z-plane is the "circle"-plane
   *
   */
  class ContactKinematicsEHDInterface : public MBSim::ContactKinematics {

      friend class EHDMesh; //TODO: should only be avoided

    public:
      virtual void updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData, int index);

      virtual void updateKinematics(const std::vector<MBSim::SingleContact> & contacts) = 0;
      // Film thickness and derivatives
      //
      // Input:
      //   sys:    Object of system
      //   y:      Point inside fluid domain (y-coordinate)
      //   e(~):   Element number in spatial discretization
      //   g(~):   Gauss point number in element corresponding to x
      //
      // Output:
      //   h1:     Distance from bearing shell center point to point
      //           on journal surface
      //   h2:     Distance from bearing shell center point to point
      //           on inner bearing shell surface
      //   h1dy:   Derivative of h1 with respect to y
      //   h2dy:   Derivative of h2 with respect to y
      virtual void Thickness(const fmatvec::VecV & x, double & h1, double & h2, double & h1dy, double & h2dy) = 0;

      // Velocities on journal and inner bearing shell surface and
      // derivatives
      //
      // Input:
      //   sys:    Object of system
      //   x:      Point inside fluid domain y or [y; z]
      //   e(~):   Element number in spatial discretization
      //   g(~):   Gauss point number in element corresponding to x
      //
      // Output:
      //   u1, v1: Velocities on journal surface
      //   u2, v2: Velocities on inner bearing shell surface
      //   v1dy:   Derivative of v1 with respect to y
      //   v2dy:   Derivative of v2 with respect to y
      virtual void Velocities(const fmatvec::VecV & x, double & u1, double & u2, double & v1, double & v2, double & v1dy, double & v2dy) = 0;

      // Normal vector
      //
      // Input:
      //   y:      Point inside fluid domain (y-coordinate)
      //
      // Output:
      //   n:       normal vectors
      //   t:       tangential vecotrs
      virtual void Normalvector(const fmatvec::VecV & x, fmatvec::Vec3 & n, fmatvec::Mat3x2 & t) = 0;

      /*!
       * \brief retrieve characteristic size for film thickness
       */
      double gethrF() {
        return hrF;
      }

      /*!
       * \brief retrieve characteristic size for fluid domain
       */
      double getxrF() {
        return xrF;
      }

      void setdimless(bool dimless_) {
        dimLess = dimless_;
      }

      bool getdimless() {
        return dimLess;
      }

      void setNumberOfPotentialContactPoints(int ncP) {
        numberOfPotentialContactPoints = ncP;
      }

    protected:
      /*!
       * \brief Flag for dimensionless description
       */
      bool dimLess;

      /*!
       * \brief Characteristic size for fluid domain
       */
      double xrF;

      /*!
       * \brief Characteristic size for film thickness
       */
      double hrF;

      /*!
       * \brief Node positions x_i^k (i: direction, k: node) for the force computation
       *
       * Example 1D: pos = [x_1^1; x_1^2; x_1^3; ...]
       * Example 2D: pos = [x_1^1; x_2^1; x_1^2; x_2^2; ...]
       */
      fmatvec::VecV pos;

      /*!
       * \brief saves the h-positions for each spatial position of pos
       */
      std::vector<fmatvec::Vec2> heightsPos;

      /*!
       * \brief saves the derivatives of the height positions with respect to the first spatial coordinate (y) of pos
       */
      std::vector<fmatvec::Vec2> dheightsdyPos;

  };
}
#endif
