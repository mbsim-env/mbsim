/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#include <config.h>

#include "contour_1s_neutral_cosserat.h"

#include <mbsim/utils/rotarymatrices.h>

namespace MBSimFlexibleBody {

  Contour1sNeutralFactory::Contour1sNeutralFactory(const std::string &name, double uMin, double uMax, bool openStructure) :
      MBSim::Contour1s(name), uMin(uMin), uMax(uMax), openStructure(openStructure)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVSpineExtrusion(0)
#endif
  {
  }

  Contour1sNeutralFactory::~Contour1sNeutralFactory() {
    /* TODO: delete of spine-extrusion not possible!
     if(openMBVSpineExtrusion)
     delete openMBVSpineExtrusion;*/
  }

  void Contour1sNeutralFactory::init(MBSim::InitStage stage) {
    if (stage == MBSim::plot) {
      updatePlotFeatures();
#ifdef HAVE_OPENMBVCPPINTERFACE

      if (getPlotFeature(openMBV) == enabled && openMBVSpineExtrusion) {
        openMBVSpineExtrusion->setName(name);
        parent->getOpenMBVGrp()->addObject(openMBVSpineExtrusion);
        if (openMBVSpineExtrusion)
          openMBVSpineExtrusion->setInitialRotation(AIK2Cardan(R->getOrientation()));
      }
#endif
    }
    Contour1s::init(stage);
  }

  void Contour1sNeutralFactory::plot(double t, double dt) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    if (getPlotFeature(openMBV) == enabled && openMBVSpineExtrusion) {

      vector<double> data;
      data.push_back(t);
      double s = uMin;
      double ds = (uMax - uMin) / (openMBVSpineExtrusion->getNumberOfSpinePoints() - 1);
      if (not openStructure)
        ds = (uMax - uMin) / (openMBVSpineExtrusion->getNumberOfSpinePoints() - 2);
      for (int i = 0; i < openMBVSpineExtrusion->getNumberOfSpinePoints(); i++) {
        ContourPointData cp(s);
        updateKinematicsForFrame(cp, position);
        Vec pos = cp.getFrameOfReference().getPosition();
        data.push_back(pos(0)); // global x-position
        data.push_back(pos(1)); // global y-position
        data.push_back(pos(2)); // global z-position
        data.push_back(0.); // local twist

        s += ds;
      }

      openMBVSpineExtrusion->append(data);
    }
#endif
  }
} /* namespace MBSimFlexibleBody */
