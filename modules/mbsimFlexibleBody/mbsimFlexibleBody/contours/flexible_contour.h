/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _FLEXIBLE_CONTOUR_H_
#define _FLEXIBLE_CONTOUR_H_

#include "mbsim/contours/contour.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {

  /*!  
   * \brief flexible planar contour
   * \author Martin Foerg
   */
  class FlexibleContour : public MBSim::Contour {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      FlexibleContour(const std::string &name="") : MBSim::Contour(name) { }

      /**
       * \brief destructor
       */
      ~FlexibleContour() override = default;

      virtual fmatvec::Vec3 evalWs_t(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalWt_t(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalWu_t(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalWv_t(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalWn_t(const fmatvec::Vec2 &zeta);

      virtual fmatvec::Vec3 evalAngularVelocity();

      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override { return findContactPairingFlexible(type0, type1); }
  };

}

#endif
