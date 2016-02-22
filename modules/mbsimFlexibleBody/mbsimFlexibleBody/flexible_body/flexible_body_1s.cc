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

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  void FlexibleBody1s::addFrame(Frame1s *frame) { 
    Body::addFrame(frame); 
  }

  Vec3 FlexibleBody1s::getPosition(double t, double s) {
    THROW_MBSIMERROR("(FlexibleBody1s::getPosition): Not implemented.");
  }

  SqrMat3 FlexibleBody1s::getOrientation(double t, double s) {
    THROW_MBSIMERROR("(FlexibleBody1s::getOrientation): Not implemented.");
  }

  Vec3 FlexibleBody1s::getWs(double t, double s) {
    THROW_MBSIMERROR("(FlexibleBody1s::getWs): Not implemented.");
  }

  void FlexibleBody1s::updatePositions(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s::updatePositions): Not implemented.");
  }

  void FlexibleBody1s::updateVelocities(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s::updateVelocities): Not implemented.");
  }

  void FlexibleBody1s::updateAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s::updateJacobians(double t, Frame1s *frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody1s::updateJacobians): Not implemented.");
  }

  void FlexibleBody1s::updateGyroscopicAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s::updateGyroscopicAccelerations): Not implemented.");
  }

}
