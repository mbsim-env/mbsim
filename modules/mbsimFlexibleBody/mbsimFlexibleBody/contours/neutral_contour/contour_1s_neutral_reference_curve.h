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

#ifndef CONTOUR_1S_NEUTRAL_REFERENCE_CURVE_H_
#define CONTOUR_1S_NEUTRAL_REFERENCE_CURVE_H_

#include "mbsimFlexibleBody/pointer.h"
#include <mbsimFlexibleBody/contours/contour_1s_neutral_factory.h>
#include <mbsimFlexibleBody/contours/neutral_contour/neutral_contour_components/neutral_nurbs_velocity_1s.h>
#include <mbsimFlexibleBody/contours/neutral_contour/neutral_contour_components/neutral_nurbs_position_1s.h>
#include <mbsimFlexibleBody/contours/neutral_contour/neutral_contour_components/neutral_nurbs_angle_1s.h>
#include <mbsimFlexibleBody/contours/neutral_contour/neutral_contour_components/neutral_nurbs_dotangle_1s.h>

namespace MBSimFlexibleBody {
  
  /*!
   * \brief class that sets up the neutral contour for the body FlexibleBody1SReferenceCurve
   *        Not that as interface the lagrange coordinate "s" is used
   */
  class Contour1sNeutralFlexibleBody1SReferenceCurve : public MBSimFlexibleBody::Contour1sNeutralFactory {
    public:
      Contour1sNeutralFlexibleBody1SReferenceCurve(const std::string &name_);
      virtual ~Contour1sNeutralFlexibleBody1SReferenceCurve();
      virtual void init(InitStage stage, const MBSim::InitConfigSet &config);
//      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::Frame::Feature ff);
//      virtual void updateJacobiansForFrame(MBSim::ContourPointData &cp, int j = 0);
      virtual MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1);

      double getuMax() const {
        return uMax;
      }
      
      double getuMin() const {
        return uMin;
      }
      
  };

} /* namespace TUM3D */
#endif
