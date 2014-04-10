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
 * Contact: martin.o.foerg@googlemail.com
 */

#include<config.h>
#include "mbsim/contours/edge.h"
#include <fmatvec/fmatvec.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Edge, MBSIMNS"Edge")

  void Edge::init(InitStage stage) {
    if (stage == MBSim::plot) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && openMBVRigidBody) {
          if (openMBVRigidBody)
            ((OpenMBV::Cuboid*) openMBVRigidBody)->setLength(0, length, 0);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  void Edge::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if (ff == velocity || ff == velocities) {
      Vec3 WrPC = cp.getFrameOfReference().getPosition() - R->getPosition();
      cp.getFrameOfReference().setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), WrPC));
    }
    if (ff == angularVelocity || ff == velocities)
      cp.getFrameOfReference().setAngularVelocity(R->getAngularVelocity());

    if (ff == position) {
      double s = cp.getLagrangeParameterPosition()(0);
      cp.getFrameOfReference().getPosition() = R->getPosition() + R->getOrientation().col(1) * s;
    }

    if (ff == firstTangent) {
      cp.getFrameOfReference().getOrientation().set(1, R->getOrientation().col(1));
    }

    if (ff!= position and ff != firstTangent and ff != velocity && ff != angularVelocity && ff != velocities)
      throw MBSimError("ERROR (RigidContour::updateKinematicsForFrame): FrameFeature not implemented!");

  }

}
