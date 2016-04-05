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
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContourFrame* Contour1sFlexible::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

  void Contour1sFlexible::resetUpToDate() {
    Contour1s::resetUpToDate();
    sOld = -1e12;
  }

  void Contour1sFlexible::updatePositions(double s) {
    static Vec3 Kt("[0;0;1]");
    Frame1s P("P",s);
    P.setParent(parent);
    Ws = P.evalOrientation().col(0);
    Wt = P.getOrientation()*Kt;
    WrOP = P.getPosition();
    sOld = s;
  }

  void Contour1sFlexible::updatePositions(ContourFrame *frame) {
    Frame1s P("P",frame->getEta());
    P.setParent(parent);
    frame->getOrientation(false).set(0,P.evalOrientation().col(1));
    frame->getOrientation(false).set(1,P.getOrientation().col(0));
    frame->getOrientation(false).set(2,-P.getOrientation().col(2));
    frame->setPosition(P.getPosition());
  }

  void Contour1sFlexible::updateVelocities(ContourFrame *frame) {
    Frame1s P("P",frame->getEta());
    P.setParent(parent);
    frame->setAngularVelocity(P.evalAngularVelocity());
    frame->setVelocity(P.getVelocity());
 }

  void Contour1sFlexible::updateAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour1sFlexible::updateAccelerations): Not implemented!");
  }

  void Contour1sFlexible::updateJacobians(ContourFrame *frame, int j) {
    Frame1s P("P",frame->getEta());
    P.setParent(parent);
    frame->setJacobianOfRotation(P.evalJacobianOfRotation(j),j);
    frame->setJacobianOfTranslation(P.getJacobianOfTranslation(j),j);
  }

  void Contour1sFlexible::updateGyroscopicAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(Contour1sFlexible::updateGyroscopicAccelerations): Not implemented!");
  }

}
