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
      LinkMechanics(name), c(0.), d(0.), alpha0(0.) {
  }

  IsotropicRotationalSpringDamper::~IsotropicRotationalSpringDamper() {
  }

  void IsotropicRotationalSpringDamper::updateh(double t, int j) {
    for (int i = 0; i < momentDir.cols(); i++)
      la(i) = -gd(i) * d - g(i) * c;

    WM[1] = Wm * la(IR);
    WM[0] = -WM[1];

    h[j][0] += frame[0]->getJacobianOfRotation(j).T() * WM[0];
    h[j][1] += frame[1]->getJacobianOfRotation(j).T() * WM[1];
  }

  void IsotropicRotationalSpringDamper::updateg(double t) {
    Wm = frame[0]->getOrientation() * momentDir; // directions of torque

    Vec r1 = frame[0]->getOrientation().col(0); // first column (tangent) of first frame
    Vec r2 = frame[1]->getOrientation().col(0); // first column (tangent) of second frame

    double alpha;

    if (r1.T() * r2 < -1 + epsroot()) {
      alpha = M_PI;

    }
    else if (r1.T() * r2 > 1 - epsroot()) {
      alpha = 0;

    }
    else
      alpha = acos(r1.T() * r2);

    Vec normal = crossProduct(r1, r2); // normal for rotation from r1 to r2

    if (nrm2(normal) < epsroot()) { // r1 and r2 parallel -> rotation less than 180°
      normal(0) = 0;
      normal(1) = 0;
      normal(2) = 0;
    }
    else
      // r1 and r2 not parallel
      normal /= nrm2(normal);

    //g(IR) = Wm.T() * normal * (alpha - alpha0); // TODO (T.S. : Perhaps you can use alpha for the force law and the normal for the direction independently.)
    g(IR) = Wm.T() * normal * alpha;

  }

  void IsotropicRotationalSpringDamper::updategd(double t) {
    Vec3 relOmega = frame[1]->getAngularVelocity() - frame[0]->getAngularVelocity();
    gd(IR) = Wm.T() * relOmega;
  }

  void IsotropicRotationalSpringDamper::init(InitStage stage) {
    if (stage == resize) {
      LinkMechanics::init(stage);
      g.resize(momentDir.cols());
      gd.resize(momentDir.cols());
      la.resize(momentDir.cols());
    }
    else if (stage == unknownStage) {
      LinkMechanics::init(stage);

      IR = Index(0, momentDir.cols() - 1);

      if (momentDir.cols())
        Wm = momentDir;
      else {
        momentDir.resize(3, 0);
        Wm.resize(3, 0);
      }
    }
    else
      LinkMechanics::init(stage);
  }

  void IsotropicRotationalSpringDamper::connect(Frame* frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void IsotropicRotationalSpringDamper::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);
    momentDir = md;

    for (int i = 0; i < md.cols(); i++)
      momentDir.col(i) = momentDir.col(i) / nrm2(md.col(i));
  }

}

