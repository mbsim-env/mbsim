/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: ghorst@amm.mw.tum.de
 */

#include <config.h>

#include "isotropic_rotational_spring_damper.h"
#include "mbsim/rigid_body.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  IsotropicRotationalSpringDamper::IsotropicRotationalSpringDamper(const string &name) :
      FloatinFrameToFrameLink(name), c(0.), d(0.), alpha0(0.) {
  }

  IsotropicRotationalSpringDamper::~IsotropicRotationalSpringDamper() {
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedSingleValuedForces(doublet) {
    la = -getGeneralizedRelativeVelocity(t) * d - getGeneralizedRelativePosition(t) * c;
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedPositions(double t) {
    Vec3 r1 = frame[0]->getOrientation(t).col(0); // first column (tangent) of first frame
    Vec3 r2 = frame[1]->getOrientation(t).col(0); // first column (tangent) of second frame

    double alpha;

    if (r1.T() * r2 < -1 + epsroot()) {
      alpha = M_PI;

    }
    else if (r1.T() * r2 > 1 - epsroot()) {
      alpha = 0;

    }
    else
      alpha = acos(r1.T() * r2);

    Vec3 normal = crossProduct(r1, r2); // normal for rotation from r1 to r2

    if (nrm2(normal) < epsroot()) { // r1 and r2 parallel -> rotation less than 180°
      normal(0) = 0;
      normal(1) = 0;
      normal(2) = 0;
    }
    else
      // r1 and r2 not parallel
      normal /= nrm2(normal);

    //g(IR) = Wm.T() * normal * (alpha - alpha0); // TODO (T.S. : Perhaps you can use alpha for the force law and the normal for the direction independently.)
    rrel = getGlobalMomentDirection(t).T() * normal * alpha;

    updrrel = false;
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedVelocities(double t) {
    vrel = getGlobalMomentDirection(t).T() * getGlobalRelativeVelocity(t);
    updvrel = false;
  }

  void IsotropicRotationalSpringDamper::setMomentDirection(const Mat3xV &md) {
    momentDir = md;

    for (int i = 0; i < md.cols(); i++)
      momentDir.col(i) = momentDir.col(i) / nrm2(md.col(i));
  }

}

