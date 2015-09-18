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
#include "mbsim/frame.h"

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

      /*!
       * \brief interface function for the mesh-positions to update the necessary entries at certain positions x (either nodes or gauss-points, or...)
       *
       * \param   x   position of the surface coordinate
       * \param   ff  the frame feature signalizes what should be computed.
       *              If all (=default) is active, the complete vector should be returned
       *              If position is active, only the position information is computed
       * \return  A vector of not specified length holding the certain information which is needed. It holds the following order
       *          h1    distance of the surfaces in first direction
       *          h2    distance of the surfaces in second direction
       *          dh1d1 spatial derivative of h1 in the first direction(=dh1dy)
       *          dh2d1 spatial derivative of h2 in the first direction(=dh2dy)
       *          u1    Velocity in first direction of the first surface
       *          u2    Velocity in second direction of the first surface
       *          v1    Velocity in first direction of the second surface
       *          v2    Velocity in second direction of the second surface
       *          dv1d1
       *          dv1d2
       * \todo: this is maybe not the best interface as it does not consider all possibilites. At the current state however
       */
      virtual fmatvec::VecV updateKinematics(const fmatvec::Vec2 & alpha, MBSim::Frame::Feature ff = MBSim::Frame::all) = 0;

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
