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

#include<config.h>
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include <vector>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBand::FlexibleBand(const string& name) :
      Contour1sFlexible(name), Cn(2, INIT, 0.), width(0.), nDist(0.) {
  }

  void FlexibleBand::setCn(const Vec& Cn_) {
    assert(Cn_.size() == 2);
    Cn = Cn_ / nrm2(Cn_);
  }

  Vec3 FlexibleBand::getPosition(double t, MBSim::ContourPointData &cp) {
    return static_cast<FlexibleBody*>(parent)->getPosition(t,cp);
  }

  Vec3 FlexibleBand::getWu(double t, MBSim::ContourPointData &cp) {
    return static_cast<FlexibleBody*>(parent)->getWu(t,cp);
  }

//  Vec3 FlexibleBand::getVelocity(double t, MBSim::ContourPointData &cp) {
//    return static_cast<FlexibleBody*>(parent)->updateVelocities(t,cp);
//    return cp.getFrameOfReference()->getVelocity(false);
//  }

//  void FlexibleBand::updateKinematicsForFrame(ContourPointData& cp, Frame::Feature ff) {
//    if (ff == Frame::firstTangent || ff == Frame::cosy || ff == Frame::position_cosy || ff == Frame::velocity_cosy || ff == Frame::velocities_cosy)
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::firstTangent);
//    if (ff == Frame::normal || ff == Frame::secondTangent || ff == Frame::cosy || ff == Frame::position_cosy || ff == Frame::velocity_cosy || ff == Frame::velocities_cosy) {
////      static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,normal);
////      static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,secondTangent);
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::normal);
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::secondTangent);
//
//      Vec WnLocal = cp.getFrameOfReference().getOrientation().col(0);
//      Vec WbLocal = cp.getFrameOfReference().getOrientation().col(2);
//      if (ff != Frame::secondTangent)
//        cp.getFrameOfReference().getOrientation().set(0, WnLocal * Cn(0) + WbLocal * Cn(1));
//      if (ff != Frame::normal)
//        cp.getFrameOfReference().getOrientation().set(2, -WnLocal * Cn(1) + WbLocal * Cn(0));
//    }
//    if (ff == Frame::position || ff == Frame::position_cosy) {
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::position);
//      cp.getFrameOfReference().getPosition() += cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
//    }
//    if (ff == Frame::angularVelocity || ff == Frame::velocities || ff == Frame::velocities_cosy) {
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::angularVelocity);
//    }
//    if (ff == Frame::velocity || ff == Frame::velocity_cosy || ff == Frame::velocities || ff == Frame::velocities_cosy) {
//      Contour1sFlexible::updateKinematicsForFrame(cp, Frame::velocity);
//      Vec3 dist = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
//      cp.getFrameOfReference().getVelocity() += crossProduct(cp.getFrameOfReference().getAngularVelocity(), dist);
//    }
//  }
//
//  void FlexibleBand::updateJacobiansForFrame(ContourPointData &cp, int j /*=0*/) {
//    Contour1sFlexible::updateJacobiansForFrame(cp);
//    Vec3 WrPC = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1); // vector from neutral line to contour surface point
//    SqrMat3 tWrPC = tilde(WrPC); // tilde matrix of above vector
//    cp.getFrameOfReference().setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation() - tWrPC * cp.getFrameOfReference().getJacobianOfRotation()); // Jacobian of translation at contour surface with standard description assuming rigid cross-section
//  }

}

