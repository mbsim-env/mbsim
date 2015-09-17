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

#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/mbsim_event.h"

namespace MBSim {

  /*!
   * \brief interface class for all contact kinematics for EHD computation
   */
  class ContactKinematicsEHDInterface : public ContactKinematics {

  };

  class CircleSolid;
  class CircleHollow;

  /**
   * \brief pairing circle outer side to circle inner side for EHD contacts
   * \author Kilian Grundl
   */
  class ContactKinematicsCircleSolidCircleHollowEHD : public ContactKinematicsEHDInterface {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData *cpData);
      /***************************************************/

      fmatvec::Vec3 getWrD();
      fmatvec::Vec3 getWrDdot();
      fmatvec::Vec3 getomegaRel();

      CircleSolid * getcirclesol();
      CircleHollow * getcirclehol();

    private:
      /**
       * \brief contour index
       */
      int icirclesol, icirclehol;

      /**
       * \brief contour classes
       */
      CircleSolid *circlesol;
      CircleHollow *circlehol;

      /*!
       * \brief distance between the center points
       */
      fmatvec::Vec3 WrD;

      /*!
       * \brief relative velocity between the center points
       */
      fmatvec::Vec3 WrDdot;

      /*!
       * \brief relative angular velocity
       */
      fmatvec::Vec3 omegaRel;

  };

}

#endif
