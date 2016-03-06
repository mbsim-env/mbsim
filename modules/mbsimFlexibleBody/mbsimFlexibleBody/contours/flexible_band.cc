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
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  void FlexibleBand::setRelativePosition(const fmatvec::Vec2 &r) {
    RrRP(1) = r(0);
    RrRP(2) = r(1);
  }

  void FlexibleBand::setRelativeOrientation(double al) {
    ARK = BasicRotAIKx(al);
  }

  void FlexibleBand::updatePositions(double t, double s) {
    Contour1sFlexible::updatePositions(t,s);
    static Vec3 Kt("[0;0;1]");
    WrOP = P.getPosition(false) + P.getOrientation(false)*RrRP;
    Wt = P.getOrientation(false)*(ARK*Kt);
  }

}
