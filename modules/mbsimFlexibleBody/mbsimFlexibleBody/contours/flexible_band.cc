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
    ARP = BasicRotAIKx(al);
  }

  Vec3 FlexibleBand::getPosition(double t, const Vec2 &zeta) {
    return static_cast<FlexibleBody1s*>(parent)->getPosition(t,zeta(0)) + static_cast<FlexibleBody1s*>(parent)->getOrientation(t,zeta(0))*RrRP;
  }

  Vec3 FlexibleBand::getWt(double t, const Vec2 &zeta) {
    static Vec3 Pt("[0;0;1]");
    return static_cast<FlexibleBody1s*>(parent)->getOrientation(t,zeta(0))*(ARP*Pt);
  }

}
