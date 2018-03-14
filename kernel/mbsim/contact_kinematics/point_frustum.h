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
   *
   * REMARK The first tangential of the contact frame of the frustum points in positive axial direction, i.e. the height direction
   */
  class ContactKinematicsPointFrustum : public ContactKinematics {
    public:
      /* INHERITED INTERFACE */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;
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
