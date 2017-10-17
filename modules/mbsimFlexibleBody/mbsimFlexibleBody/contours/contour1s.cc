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

#include <config.h>

#include "mbsimFlexibleBody/contours/contour1s.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/rotarymatrices.h"

#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;

namespace MBSimFlexibleBody {

  void Contour1s::init(InitStage stage, const MBSim::InitConfigSet &config) {

    if (stage == plotting) {
      if(plotFeature[MBSim::openMBV] and openMBVSpineExtrusion) {
        openMBVSpineExtrusion->setName(name);
        parent->getOpenMBVGrp()->addObject(openMBVSpineExtrusion);
//        openMBVSpineExtrusion->setInitialRotation(AIK2Cardan(R->getOrientation()));
      }
    }
    Contour::init(stage, config);
  }

  Vec3 Contour1s::evalKt(const fmatvec::Vec2 &zeta) {
    static Vec3 Kt("[0;0;1]");
    return Kt;
  }

  void Contour1s::plot() {
    if(plotFeature[MBSim::openMBV] and openMBVSpineExtrusion) {

      vector<double> data;
      data.push_back(getTime());
      double s = etaNodes[0];
      double ds = (etaNodes[etaNodes.size()-1] - etaNodes[0]) / (openMBVSpineExtrusion->getNumberOfSpinePoints() - 1);
      Vec2 zeta;

      // TODO: for open structure one could think of using one more element to print the closure a littel prettier...
//      if (not openStructure)
//        ds = (uMax - uMin) / (openMBVBody->getNumberOfSpinePoints() - 2);
      for (int i = 0; i < openMBVSpineExtrusion->getNumberOfSpinePoints() - 1; i++) {
        zeta(0) = s;
        Vec3 pos = evalPosition(zeta);
        data.push_back(pos(0)); // global x-position
        data.push_back(pos(1)); // global y-position
        data.push_back(pos(2)); // global z-position
        data.push_back(0.); // local twist

        s += ds;
      }
      // Avoid s-parameters to be longer than etaNodes[etaNodes.size()-1]!
      zeta(0) = etaNodes[etaNodes.size()-1];
      Vec3 pos = evalPosition(zeta);
      data.push_back(pos(0)); // global x-position
      data.push_back(pos(1)); // global y-position
      data.push_back(pos(2)); // global z-position
      data.push_back(0.); // local twist

      openMBVSpineExtrusion->append(data);
    }
    Contour::plot();
  }

}
