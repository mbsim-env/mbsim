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
#include "mbsimFlexibleBody/frames/floating_contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  const Vec3& FloatingContourFrame::getGlobalRelativePosition(double t) {
    if(updatePos) updatePositions(t);
    return WrRP; 
  }

  void FloatingContourFrame::updatePositions(double t) {
    parent->updatePositions(t,this);
    R->setParameter(getEta());
    WrRP = getPosition(false) - R->getPosition(t);
    updatePos = false;
  }

  void FloatingContourFrame::updateVelocities(double t) {
    R->setParameter(getEta());
    setAngularVelocity(R->getAngularVelocity(t));
    setVelocity(R->getVelocity(t) + crossProduct(R->getAngularVelocity(), getGlobalRelativePosition(t))); 
    updateVel = false;
  }

  void FloatingContourFrame::updateAccelerations(double t) {
    R->setParameter(getEta());
    setAngularAcceleration(R->getAngularAcceleration(t));
    setAcceleration(R->getAcceleration(t) + crossProduct(R->getAngularAcceleration(), getGlobalRelativePosition(t)) + crossProduct(R->getAngularVelocity(t), crossProduct(R->getAngularVelocity(t), getGlobalRelativePosition(t)))); 
    updateAcc = true;
  }

  void FloatingContourFrame::updateJacobians(double t, int j) {
    R->setParameter(getEta());
     setJacobianOfTranslation(R->getJacobianOfTranslation(t,j) - tilde(getGlobalRelativePosition(t))*R->getJacobianOfRotation(t,j),j);
    setJacobianOfRotation(R->getJacobianOfRotation(j),j);
   updateJac[j] = false;
  }

  void FloatingContourFrame::updateGyroscopicAccelerations(double t) {
    R->setParameter(getEta());
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(t) + crossProduct(R->getGyroscopicAccelerationOfRotation(t),getGlobalRelativePosition(t)) + crossProduct(R->getAngularVelocity(t),crossProduct(R->getAngularVelocity(t),getGlobalRelativePosition(t))));
    setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation());
   updateGA = false;
  }

}
