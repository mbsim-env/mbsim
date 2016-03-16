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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_2s.h"
#include "mbsimFlexibleBody/frames/frame_2s.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

 void FlexibleBody2s::init(InitStage stage) {
    if(stage==plotting) {
//#ifdef HAVE_OPENMBVCPPINTERFACE
//      ((OpenMBV::SpineExtrusion*)openMBVBody.get())->setInitialRotation(AIK2Cardan(R->getOrientation(0.)));
//#endif
      FlexibleBodyContinuum<Vec2>::init(stage);
    }
    else
      FlexibleBodyContinuum<Vec2>::init(stage);
  }

  void FlexibleBody2s::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
//#ifdef HAVE_OPENMBVCPPINTERFACE
//      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
//        vector<double> data;
//        data.push_back(t);
//        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints()-2);
//        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints(); i++) {
//          Vec3 pos = getPosition(t,ds*i);
//          data.push_back(pos(0)); // global x-position
//          data.push_back(pos(1)); // global y-position
//          data.push_back(pos(2)); // global z-position
//          data.push_back(getLocalTwist(t,ds*i)); // local twist
//        }
//        ((OpenMBV::SpineExtrusion*)openMBVBody.get())->append(data);
//      }
//#endif
    }
    FlexibleBodyContinuum<Vec2>::plot(t,dt);
  }

  void FlexibleBody2s::addFrame(Frame2s *frame) { 
    Body::addFrame(frame); 
  }

  void FlexibleBody2s::updatePositions(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s::updatePositions): Not implemented.");
  }

  void FlexibleBody2s::updateVelocities(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s::updateVelocities): Not implemented.");
  }

  void FlexibleBody2s::updateAccelerations(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s::updateJacobians(double t, Frame2s *frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody2s::updateJacobians): Not implemented.");
  }

  void FlexibleBody2s::updateGyroscopicAccelerations(double t, Frame2s *frame) {
    THROW_MBSIMERROR("(FlexibleBody2s::updateGyroscopicAccelerations): Not implemented.");
  }

}
