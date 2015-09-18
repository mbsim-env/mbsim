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

#ifndef _CONTACT_KINEMATICS_CIRCLESOLID_CIRCLEHOLLOW_EHD_H_
#define _CONTACT_KINEMATICS_CIRCLESOLID_CIRCLEHOLLOW_EHD_H_

#include "contact_kinematics_ehd_interface.h"

namespace MBSim {
  class Frustum;
}

namespace MBSimEHD {

  /**
   * \brief pairing circle outer side to circle inner side for EHD contacts
   * \author Kilian Grundl
   * \author Michael Hofer
   *
   * Refer to
   *  [1] Michael Hofer: Isotherme elastohydrodynamicse Kontake in Mehrkörpersystemen (Semesterarbeit at Angewandte Mechanik of TU München)
   */
  class ContactKinematicsCylinderSolidCylinderHollowEHD : public ContactKinematicsEHDInterface {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<MBSim::Contour*> &contour);
      virtual void updateKinematics(const std::vector<MBSim::SingleContact> & contacts);
      virtual fmatvec::VecV updateKinematics(const fmatvec::Vec2 & alpha, MBSim::Frame::Feature ff = MBSim::Frame::all);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData *cpData);
      /***************************************************/

      fmatvec::Vec3 getWrD();
      fmatvec::Vec3 getWrDdot();
      fmatvec::Vec3 getomegaRel();

      MBSim::Frustum * getcirclesol();
      MBSim::Frustum * getcirclehol();

    private:
      /**
       * \brief contour index
       */
      int isolid, ihollow;

      /**
       * \brief contour classes
       */
      MBSim::Frustum *solid;
      MBSim::Frustum *hollow;

      /*!
       * \brief the radius of the solid
       */
      double rSolid;

      /*
       * \brief the radius of the hollow
       */
      double rHollow;

      /*!
       * \brief distance between the center points
       *
       * It is "e" in [1] in world coordinates
       */
      fmatvec::Vec3 IrC1C2;

      /*!
       * \brief relative velocity between the center points
       */
      fmatvec::Vec3 IvC1C2;

      /*!
       * \brief relative angular velocity
       */
      fmatvec::Vec3 omegaRel;

      /*!
       * \brief Angle coordinate
       */
      double phi;

      /*!
       * \brief rotation matrix from F to K
       */
      fmatvec::SqrMat3 AKF;

      /*!
       * \brief rotation matrix from F to I
       */
      fmatvec::SqrMat3 AIF;

      /*
       * \brief Radial eccentricity in coordinate system F
       */
      double er;

      /*!
       * \brief Tangential eccentricity in coordinate system F
       */
      double et;

      /*
       * \brief Auxiliary length variable
       */
      double r1;

    private:
      // Radial and tangential eccentricity in coordinate system F
      // and auxiliary length variable
      //
      // Input:
      //   y:      Coordinate of fluid domain
      void Eccentricity(const double & y);

  };

}

#endif
