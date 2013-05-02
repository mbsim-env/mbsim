/* Copyright (C) 2004-2009 MBSim Development Team
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
 *          rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#ifndef _CONTACT_KINEMATICS_POINT_FRUSTUM_H_
#define _CONTACT_KINEMATICS_POINT_FRUSTUM_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Point;
  class Frustum;

  /**
   * \brief pairing point to frustum surface
   * \author Martin Foerg
   * \author Thorsten Schindler
   * \date 2009-04-02 some comments (Thorsten Schindler)
   * \date 2009-07-08 new interface (Thorsten Schindler)
   * \date 2012-10-09 update kinematics (Kilian Grundl)
   *
   * REMARK The first tangential of the contact frame of the frustum points in positive axial direction, i.e. the height direction
   */
  class ContactKinematicsPointFrustum : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, ContourPointData* cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, ContourPointData *cpData) { throw MBSimError("ERROR(ContactKinematicsPointFrustum:updatewb): Not implemented!"); }
      /***************************************************/

    protected:
      /**
       * \brief contour index
       */
      int ipoint, ifrustum;

      /**
       * \brief contour classes
       */
      Point *point;
      Frustum *frustum;
  };

}

#endif /* _CONTACT_KINEMATICS_POINT_FRUSTUM_H_ */

