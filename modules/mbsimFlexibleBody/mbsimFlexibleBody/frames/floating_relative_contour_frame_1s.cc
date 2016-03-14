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
#include "mbsimFlexibleBody/frames/floating_relative_contour_frame_1s.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FloatingRelativeContourFrame1s::updatePositions(double t) {
    parent->updatePositions(t,this);
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    WrRP = getPosition(false) - R->getPosition(t);
    updatePos = false;
  }

  void FloatingRelativeContourFrame1s::updateVelocities(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateVelocities(t);
  }

  void FloatingRelativeContourFrame1s::updateAccelerations(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateAccelerations(t);
  }

  void FloatingRelativeContourFrame1s::updateJacobians(double t, int j) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateJacobians(t,j);
  }

  void FloatingRelativeContourFrame1s::updateGyroscopicAccelerations(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateGyroscopicAccelerations(t);
  }

}
