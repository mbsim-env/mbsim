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
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/floating_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FloatingFrame::updatePositions(double t) { 
    parent->updatePositions(t);
    updatePos = false;
  }

  void FloatingFrame::updateVelocities(double t) { 
    contour->updateVelocities(t,this);
    //WvP = contour->getVelocity(t,zeta);
    //WomegaP = contour->getAngularVelocity(t,zeta);
    updateVel = false;
  }

  void FloatingFrame::updateAccelerations(double t) { 
    WaP = contour->getAcceleration(t,zeta);
    WpsiP = contour->getAngularAcceleration(t,zeta);
    updateAcc = true;
  }

  void FloatingFrame::updateJacobians(double t, int j) {
    WJP[j] = contour->getJacobianOfTranslation(t,zeta,j);
    WJR[j] = contour->getJacobianOfRotation(t,zeta,j);
    updateJac[j] = false;
  }

  void FloatingFrame::updateGyroscopicAccelerations(double t) {
    WjP = contour->getGyroscopicAccelerationOfTranslation(t,zeta);
    WjR = contour->getGyroscopicAccelerationOfRotation(t,zeta);
    updateGA = false;
  }

}

