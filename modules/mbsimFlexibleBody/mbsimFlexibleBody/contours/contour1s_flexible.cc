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

#include "contour1s_flexible.h"
#include "mbsimFlexibleBody/frames/floating_contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContourFrame* Contour1sFlexible::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    Frame1s *bodyFrame = new Frame1s(name);
    static_cast<FlexibleBody1s*>(parent)->addFrame(bodyFrame);
    frame->setFrameOfReference(bodyFrame);
    return frame;
  }

  Vec3 Contour1sFlexible::getPosition(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour1sFlexible::getPosition): Not implemented.");
  }

  Vec3 Contour1sFlexible::getWs(double t, const Vec2 &zeta) {
    return static_cast<FlexibleBody1s*>(parent)->getWs(t,zeta(0));
  }

  Vec3 Contour1sFlexible::getWt(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(Contour1sFlexible::getWt): Not implemented.");
  }

}
