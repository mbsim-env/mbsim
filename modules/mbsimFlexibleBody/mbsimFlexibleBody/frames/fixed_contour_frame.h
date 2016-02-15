/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _FIXED_CONTOUR_FRAME_H__
#define _FIXED_CONTOUR_FRAME_H__

#include "mbsimFlexibleBody/frames/contour_frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief cartesian frame on contour parameters of flexible bodies
   * \author Kilian Grundl
   */
  class FixedContourFrame : public ContourFrame {

    public:
      FixedContourFrame(const std::string &name = "dummy", const fmatvec::Vec2 &zeta = fmatvec::Vec2()) : ContourFrame(name,zeta) { }

      std::string getType() const { return "FixedContourFrame"; }

      void updatePositions(double t);
      void updateVelocities(double t);
      void updateAccelerations(double t);
      void updateJacobians(double t, int j=0);
      void updateGyroscopicAccelerations(double t);
  };

}

#endif /* _FRAME_H_ */

