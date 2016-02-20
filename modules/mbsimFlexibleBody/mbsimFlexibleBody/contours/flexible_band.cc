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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/frames/contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  void FlexibleBand::init(InitStage stage) {
    if(stage==preInit) {
      sign = tFlipped?-1:1;
      Contour1sFlexible::init(stage);
    }
    else
      Contour1sFlexible::init(stage);
  }

  Vec3 FlexibleBand::getPosition(double t, const Vec2 &zeta) {
    return static_cast<FlexibleBody*>(parent)->getPosition(t,zeta) + nDist*getWn(t,zeta);
  }

  Vec3 FlexibleBand::getWt(double t, const Vec2 &zeta) {
    return sign*static_cast<FlexibleBody*>(parent)->getFrameOfReference()->getOrientation().col(2);
  }

  void FlexibleBand::updatePositions(double t, ContourFrame *frame) {
    Contour1sFlexible::updatePositions(t,frame);
    frame->getPosition(false) += nDist*frame->getOrientation(false).col(0);
  }

  void FlexibleBand::updateVelocities(double t, ContourFrame *frame) {
    static_cast<FlexibleBody*>(parent)->updateVelocities(t,frame);
    Contour1sFlexible::updateVelocities(t,frame);
    frame->getVelocity(false) += crossProduct(frame->getAngularVelocity(false), frame->getOrientation(false).col(0)*nDist);
  }

  void FlexibleBand::updateAccelerations(double t, ContourFrame *frame) {
    throw;
    Contour1sFlexible::updateAccelerations(t,frame);
  }

  void FlexibleBand::updateJacobians(double t, ContourFrame *frame, int j) {
    Contour1sFlexible::updateJacobians(t,frame,j);
    frame->getJacobianOfTranslation(j,false) -= tilde(frame->getOrientation(false).col(0)*nDist)*frame->getJacobianOfRotation(j,false);
  }

  void FlexibleBand::updateGyroscopicAccelerations(double t, ContourFrame *frame) {
    throw;
    Contour1sFlexible::updateGyroscopicAccelerations(t,frame);
  }

}
