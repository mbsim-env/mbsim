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
#include "nurbs_frame_2s.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void NurbsFrame2s::updatePositions(double t) {
    static_cast<NurbsDisk2s*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void NurbsFrame2s::updateVelocities(double t) {
    static_cast<NurbsDisk2s*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void NurbsFrame2s::updateAccelerations(double t) {
    static_cast<NurbsDisk2s*>(parent)->updateAccelerations(t,this);
    updateAcc = true;
  }

  void NurbsFrame2s::updateJacobians(double t, int j) {
    static_cast<NurbsDisk2s*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void NurbsFrame2s::updateGyroscopicAccelerations(double t) {
    static_cast<NurbsDisk2s*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

}
