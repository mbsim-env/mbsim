/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void Contour1sAnalytical::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) {
      cp.getFrameOfReference().getOrientation().col(0)= funcCrPC->computeN(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(1)= funcCrPC->computeT(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(2)= -funcCrPC->computeB(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation() = cp.getFrameOfReference().getOrientation() * R.getOrientation();
    }
    if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().getPosition() = R.getPosition() + R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0));
    if(ff==velocity || velocity_cosy || ff==velocities || velocities_cosy || ff==all) cp.getFrameOfReference().getVelocity() = R.getVelocity() + crossProduct(R.getAngularVelocity(),R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0)));
    if(ff==angularVelocity || ff==velocities || velocities_cosy || ff==all) throw new MBSimError("ERROR (Contour1sAnalytical::updateKinematicsForFrame): Angular velocity not implemented!");
  }

}

