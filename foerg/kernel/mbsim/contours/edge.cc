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
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Edge, MBSIM%"Edge")

  void Edge::init(InitStage stage) {
    if (stage == plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && openMBVRigidBody) {
          if (openMBVRigidBody)
            static_pointer_cast<OpenMBV::Cuboid>(openMBVRigidBody)->setLength(0, length, 0);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  void Edge::updateKinematicsForFrame(ContourPointData &cp, Frame::Feature ff) {
    if (ff == Frame::velocity || ff == Frame::velocities) {
      Vec3 WrPC = cp.getFrameOfReference().getPosition() - R->getPosition();
      cp.getFrameOfReference().setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), WrPC));
    }
    if (ff == Frame::angularVelocity || ff == Frame::velocities)
      cp.getFrameOfReference().setAngularVelocity(R->getAngularVelocity());

    if (ff == Frame::position) {
      double s = cp.getLagrangeParameterPosition()(0);
      cp.getFrameOfReference().getPosition() = R->getPosition() + R->getOrientation().col(1) * s;
    }

    if (ff == Frame::firstTangent) {
      cp.getFrameOfReference().getOrientation().set(1, R->getOrientation().col(1));
    }

    if (ff!= Frame::position and ff != Frame::firstTangent and ff != Frame::velocity && ff != Frame::angularVelocity && ff != Frame::velocities)
      THROW_MBSIMERROR("(RigidContour::updateKinematicsForFrame): Frame::Feature not implemented!");

  }

}
