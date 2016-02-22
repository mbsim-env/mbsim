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
#include "mbsimFlexibleBody/frames/floating_contour_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContourFrame* FlexibleBand::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    static int n=1;
    stringstream s;
    s << "P" << n++;
    Frame1s *bodyFrame = new Frame1s(s.str());
    static_cast<FlexibleBody1s21RCM*>(parent)->addFrame(bodyFrame);
   frame->setFrameOfReference(bodyFrame);
    return frame;
  }

  void FlexibleBand::setRelativePosition(const fmatvec::Vec2 &r) {
    RrRP(1) = r(0);
    RrRP(2) = r(1);
  }

  void FlexibleBand::setRelativeOrientation(double al) {
    ARP = BasicRotAIKx(al);
  }

  Vec3 FlexibleBand::getPosition(double t, const Vec2 &zeta) {
    return static_cast<FlexibleBody1s21RCM*>(parent)->getPosition(t,zeta(0)) + static_cast<FlexibleBody1s21RCM*>(parent)->getOrientation(t,zeta(0))*RrRP;
  }

  Vec3 FlexibleBand::getWs(double t, const Vec2 &zeta) {
    return static_cast<FlexibleBody1s21RCM*>(parent)->getWs(t,zeta(0));
  }

  Vec3 FlexibleBand::getWu(double t, const fmatvec::Vec2 &zeta) {
    return static_cast<FlexibleBody1s21RCM*>(parent)->getWu(t,zeta(0));
  }

  Vec3 FlexibleBand::getWt(double t, const Vec2 &zeta) {
    static Vec3 Pt("[0;0;1]");
    return static_cast<FlexibleBody1s21RCM*>(parent)->getOrientation(t,zeta(0))*(ARP*Pt);
  }

  void FlexibleBand::updatePositions(double t, ContourFrame *frame) {
    throw;
  }

  void FlexibleBand::updateVelocities(double t, ContourFrame *frame) {
    throw;
  }

  void FlexibleBand::updateAccelerations(double t, ContourFrame *frame) {
    throw;
  }

  void FlexibleBand::updateJacobians(double t, ContourFrame *frame, int j) {
    throw;
  }

  void FlexibleBand::updateGyroscopicAccelerations(double t, ContourFrame *frame) {
    throw;
  }

}
