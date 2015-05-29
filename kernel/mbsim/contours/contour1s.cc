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
 * Contact: thschindler@users.berlios.de
 */

#include <config.h>

#include "contour1s.h"

#include <mbsim/utils/rotarymatrices.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Contour1s::Contour1s(const std::string &name) :
      ContourContinuum<double>(name), diameter(0.)

  {
  }

  void Contour1s::init(InitStage stage) {

    if (stage == plotting) {
      updatePlotFeatures();
#ifdef HAVE_OPENMBVCPPINTERFACE

      if (getPlotFeature(openMBV) == enabled && openMBVSpineExtrusion) {
        openMBVSpineExtrusion->setName(name);
        parent->getOpenMBVGrp()->addObject(openMBVSpineExtrusion);
        openMBVSpineExtrusion->setInitialRotation(AIK2Cardan(R->getOrientation()));
      }
#endif
    }
    else if (stage == unknownStage) {
      if (nodes.size() == 0) {
        int n = 10; //using ten nodes
        double uMin = as;
        double uMax = ae;
        setAlphaStart(uMin);
        setAlphaEnd(uMax);

        double step = (uMax - uMin) / n;
        for (int i = 0; i <= n; i++) {
          nodes.push_back(i * step);
        }
      }
    }
    ContourContinuum<double>::init(stage);
  }

  void Contour1s::plot(double t, double dt) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    if (getPlotFeature(openMBV) == enabled && openMBVSpineExtrusion) {

      vector<double> data;
      data.push_back(t);
      double s = as;
      double ds = (ae - as) / (openMBVSpineExtrusion->getNumberOfSpinePoints() - 1);

      // TODO: for open structure one could think of using one more element to print the closure a littel prettier...
//      if (not openStructure)
//        ds = (uMax - uMin) / (openMBVBody->getNumberOfSpinePoints() - 2);
      for (int i = 0; i < openMBVSpineExtrusion->getNumberOfSpinePoints() - 1; i++) {
        ContourPointData cp(s);
        Vec3 pos = cp.getFrameOfReference().getPosition(t);
        data.push_back(pos(0)); // global x-position
        data.push_back(pos(1)); // global y-position
        data.push_back(pos(2)); // global z-position
        data.push_back(0.); // local twist

        s += ds;
      }
      // Avoid s-parameters to be longer than ae!
      ContourPointData cp(ae);
      Vec3 pos = cp.getFrameOfReference().getPosition(t);
      data.push_back(pos(0)); // global x-position
      data.push_back(pos(1)); // global y-position
      data.push_back(pos(2)); // global z-position
      data.push_back(0.); // local twist

      s += ds;

      openMBVSpineExtrusion->append(data);
    }
#endif
  }

}

