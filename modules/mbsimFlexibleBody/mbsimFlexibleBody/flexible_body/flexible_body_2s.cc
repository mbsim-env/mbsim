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

 void FlexibleBody2s::init(InitStage stage, const InitConfigSet &config) {
    if(stage==plotting) {
//      ((OpenMBV::SpineExtrusion*)openMBVBody.get())->setInitialRotation(AIK2Cardan(R->evalOrientation()));
      FlexibleBodyContinuum<Vec2>::init(stage, config);
    }
    else
      FlexibleBodyContinuum<Vec2>::init(stage, config);
  }

  void FlexibleBody2s::plot() {
//    if(plotFeature[openMBV] and openMBVBody) {
//        vector<double> data;
//        data.push_back(getTime());
//        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints()-2);
//        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody.get())->getNumberOfSpinePoints(); i++) {
//          Vec3 pos = getPosition(ds*i);
//          data.push_back(pos(0)); // global x-position
//          data.push_back(pos(1)); // global y-position
//          data.push_back(pos(2)); // global z-position
//          data.push_back(getLocalTwist(ds*i)); // local twist
//        }
//        ((OpenMBV::SpineExtrusion*)openMBVBody.get())->append(data);
//      }
    FlexibleBodyContinuum<Vec2>::plot();
  }

  void FlexibleBody2s::addFrame(Frame2s *frame) { 
    Body::addFrame(frame); 
  }

  void FlexibleBody2s::updatePositions(Frame2s *frame) {
    throwError("(FlexibleBody2s::updatePositions): Not implemented.");
  }

  void FlexibleBody2s::updateVelocities(Frame2s *frame) {
    throwError("(FlexibleBody2s::updateVelocities): Not implemented.");
  }

  void FlexibleBody2s::updateAccelerations(Frame2s *frame) {
    throwError("(FlexibleBody2s::updateAccelerations): Not implemented.");
  }

  void FlexibleBody2s::updateJacobians(Frame2s *frame, int j) {
    throwError("(FlexibleBody2s::updateJacobians): Not implemented.");
  }

  void FlexibleBody2s::updateGyroscopicAccelerations(Frame2s *frame) {
    throwError("(FlexibleBody2s::updateGyroscopicAccelerations): Not implemented.");
  }

}
