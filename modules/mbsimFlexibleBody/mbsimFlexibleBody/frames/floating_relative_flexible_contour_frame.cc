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
#include "mbsimFlexibleBody/frames/floating_relative_flexible_contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  FloatingRelativeFlexibleContourFrame::FloatingRelativeFlexibleContourFrame(const std::string &name, Contour *contour) : FloatingRelativeContourFrame(name) {
    P.setContourOfReference(contour);
    setFrameOfReference(&P);
  }

  void FloatingRelativeFlexibleContourFrame::resetUpToDate() {
    FloatingRelativeContourFrame::resetUpToDate();
    P.resetUpToDate();
  }

  void FloatingRelativeFlexibleContourFrame::updatePositions(double t) {
    parent->updatePositions(t,this);
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    WrRP = getPosition(false) - R->getPosition(t);
    updatePos = false;
  }

  void FloatingRelativeFlexibleContourFrame::updateVelocities(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateVelocities(t);
  }

  void FloatingRelativeFlexibleContourFrame::updateAccelerations(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateAccelerations(t);
  }

  void FloatingRelativeFlexibleContourFrame::updateJacobians(double t, int j) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateJacobians(t,j);
  }

  void FloatingRelativeFlexibleContourFrame::updateGyroscopicAccelerations(double t) {
    static_cast<ContourFrame*>(R)->setZeta(getZeta());
    FloatingRelativeContourFrame::updateGyroscopicAccelerations(t);
  }

}
