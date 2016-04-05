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
 * Created on: Aug 26, 2014
 * Contact: kilian.grundl@gmail.com
 */

#include "config.h"
#include "contour_1s_neutral_reference_curve.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1S_reference_curve.h"

#include <mbsimFlexibleBody/flexible_body.h>
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

using namespace MBSim;

namespace MBSimFlexibleBody {
  
  Contour1sNeutralFlexibleBody1SReferenceCurve::Contour1sNeutralFlexibleBody1SReferenceCurve(const std::string &name_) :
      Contour1sNeutralFactory(name_) {

  }
  
  Contour1sNeutralFlexibleBody1SReferenceCurve::~Contour1sNeutralFlexibleBody1SReferenceCurve() {
  }

  void Contour1sNeutralFlexibleBody1SReferenceCurve::init(InitStage stage) {

    if (stage == preInit) {
      uMin = 0;
      uMax = static_cast<FlexibleBody1SReferenceCurve*>(this->parent)->getlength();
    }
    else if (stage == resize) {
      // construct contourPoint for translation nodes

    }
    else if (stage == worldFrameContourLocation) {
      R->getOrientation() = (static_cast<FlexibleBody*>(parent))->getFrameOfReference()->getOrientation();
      R->getPosition() = (static_cast<FlexibleBody*>(parent))->getFrameOfReference()->getPosition();
    }
    else if (stage == unknownStage) {
    }

    Contour1sNeutralFactory::init(stage);
  }

//  void Contour1sNeutralFlexibleBody1SReferenceCurve::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::Frame::Feature ff) {
//    FlexibleBody1SReferenceCurve * parent = static_cast<FlexibleBody1SReferenceCurve*>(this->parent);
//    double tangentialPos = cp.getLagrangeParameterPosition()(0) + parent->gets();
//    if (ff == Frame::position || ff == Frame::position_cosy || ff == Frame::all)
//      cp.getFrameOfReference().setPosition(parent->computer(tangentialPos, 0, 0));
//    if (ff == Frame::velocity || ff == Frame::velocity_cosy || ff == Frame::velocities || ff == Frame::velocities_cosy || ff == Frame::all)
//      cp.getFrameOfReference().setVelocity(parent->computev(tangentialPos));
//    if (ff == Frame::normal || ff == Frame::firstTangent || ff == Frame::secondTangent || ff == Frame::cosy || ff == Frame::position_cosy || ff == Frame::velocity_cosy || ff == Frame::velocities_cosy || ff == Frame::all) {
//      Vec3 rdXi = parent->computer(tangentialPos, 1, 0);
//      Vec3 t = rdXi / nrm2(rdXi);
//      // the tangent does not change -> the normal vector is not uniquely defined! --> chose normal that point outwards...
//      Vec3 n = crossProduct(t, - parent->b);
//      cp.getFrameOfReference().getOrientation().set(0, n);
//      cp.getFrameOfReference().getOrientation().set(1, t);
//      cp.getFrameOfReference().getOrientation().set(2, crossProduct(n, t));
//    }
//  }
//
//  void Contour1sNeutralFlexibleBody1SReferenceCurve::updateJacobiansForFrame(MBSim::ContourPointData &cp, int j) {
//
//    FlexibleBody1SReferenceCurve * parent = static_cast<FlexibleBody1SReferenceCurve*>(this->parent);
//    double tangentialPos = cp.getLagrangeParameterPosition()(0) + parent->gets();
//    cp.getFrameOfReference().setJacobianOfTranslation(parent->getFrameOfReference()->getOrientation() * parent->computeP(tangentialPos, 0), j);
//
//    //TODO: anything important concerning the Jacobian of rotation (now the ring is not able to transfer momements, but is that one needed?)
//  }
  
  MBSim::ContactKinematics * Contour1sNeutralFlexibleBody1SReferenceCurve::findContactPairingWith(std::string type0, std::string type1) {
    return findContactPairingFlexible(type0.c_str(), type1.c_str());
  }

}
/* namespace MBSimFlexibleBody */
