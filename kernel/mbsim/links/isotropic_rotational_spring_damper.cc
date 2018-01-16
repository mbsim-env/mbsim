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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "isotropic_rotational_spring_damper.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  IsotropicRotationalSpringDamper::IsotropicRotationalSpringDamper(const string &name) : FixedFrameLink(name), func(NULL) {
  }

  IsotropicRotationalSpringDamper::~IsotropicRotationalSpringDamper() {
  }

  void IsotropicRotationalSpringDamper::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      iF = RangeV(0, -1);
      iM = RangeV(0, 0);
      DF.resize(0);
      lambdaM.resize(1);
    }
    FixedFrameLink::init(stage, config);
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedPositions() {
    rrel(0) = acos(std::max(-1.,std::min(1.,evalGlobalRelativeOrientation()(0,0))));
    updrrel = false;
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedVelocities() {
    vrel = evalGlobalMomentDirection().T() * evalGlobalRelativeAngularVelocity();
    updvrel = false;
  }

  void IsotropicRotationalSpringDamper::updateForceDirections() {
    n(1) = -evalGlobalRelativeOrientation()(2,0);
    n(2) = AK0K1(1,0);
    double nrm = nrm2(n);
    if (nrm >= epsroot)
//      n.init(0);
//    else
      n /= nrm;
    DM = frame[0]->getOrientation() * n;
    updDF = false;
  }

  void IsotropicRotationalSpringDamper::updatelaM() {
    lambdaM(0) = -(*func)(evalGeneralizedRelativePosition()(0),evalGeneralizedRelativeVelocity()(0));
    updlaM = false;
  }


}
