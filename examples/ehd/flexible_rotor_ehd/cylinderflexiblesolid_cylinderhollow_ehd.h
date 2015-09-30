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

#ifndef _CONTACT_KINEMATICS_CYLINDERFLEXIBLESOLID_CYLINDERHOLLOW_EHD_H_
#define _CONTACT_KINEMATICS_CYLINDERFLEXIBLESOLID_CYLINDERHOLLOW_EHD_H_

#include <mbsimEHD/contact_kinematics/contact_kinematics_ehd_interface.h>
#include <mbsim/frame.h>

namespace MBSim {
  class Frustum;
}

namespace MBSimFlexibleBody {
  class CylinderFlexible;
}

namespace MBSimEHD {

  /**
   * \brief pairing cylinderical flexible contour at inner side and a rigid cylinder at the outer side for EHD contacts
   *
   * As "master"-contour the rigid cylinder is defined. It defines the mesh and therefore also the contact positions, i.e. the nodes for the mesh.
   * For all of theses points a closest contact point is found on the flexible countour using a search algorithm. The kinematics at these points are used for the EHD-contact computation.
   * \author Kilian Grundl
   */
  class ContactKinematicsCylinderFlexibleSolidCylinderHollowEHD : public ContactKinematicsEHDInterface {
    public:

      virtual ~ContactKinematicsCylinderFlexibleSolidCylinderHollowEHD();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<MBSim::Contour*> &contour);
      virtual void updateKinematics(const std::vector<MBSim::SingleContact> & contacts);
      virtual fmatvec::VecV updateKinematics(const fmatvec::Vec2 & alpha, MBSim::Frame::Feature ff = MBSim::Frame::all);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData *cpData);
      /***************************************************/

      MBSimFlexibleBody::CylinderFlexible * getSolidContour();
      MBSim::Frustum * getHollowContour();

    private:
      /**
       * \brief contour index
       */
      int isolid, ihollow;

      /**
       * \brief contour classes
       */
      MBSimFlexibleBody::CylinderFlexible *solid;
      MBSim::Frustum *hollow;

      /*!
       * \brief updates the positions of the contact points into the two contour point datas
       */
      virtual void updatePointKinematics(MBSim::ContourPointData & cpSolid, MBSim::ContourPointData & cpHollow);

      /*!
       * \brief the radius of the solid
       */
      double rSolid;

      /*
       * \brief the radius of the hollow
       */
      double rHollow;

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

      /*!
       * \brief node positions for global search
       */
      fmatvec::VecV nodes;
  };

}

#endif
