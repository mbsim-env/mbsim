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

#include <config.h>
#include "fixed_contour_frame.h"
#include "mbsim/contours/contour.h"

namespace MBSim {

  void FixedContourFrame::updatePositions(double t) {
    contour->updatePositions(t,this);
    updatePos = false;
  }

  void FixedContourFrame::updateVelocities(double t) {
    contour->updateVelocities(t,this);
    updateVel = false;
  }

  void FixedContourFrame::updateAccelerations(double t) {
    contour->updateAccelerations(t,this);
    updateAcc = true;
  }

  void FixedContourFrame::updateJacobians(double t, int j) {
    contour->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void FixedContourFrame::updateGyroscopicAccelerations(double t) {
    contour->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

}
