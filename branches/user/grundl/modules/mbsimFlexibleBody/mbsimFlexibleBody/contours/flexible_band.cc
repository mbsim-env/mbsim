/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/flexible_body.h"
#include <vector>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBand::FlexibleBand(const string& name) :
      Contour1sFlexible(name), Cn(2, INIT, 0.), width(0.), nDist(0.), openStructure(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVBody(0), openMBVGrp(0), openMBVNeturalFibre(0)
#endif
  {
  }
  FlexibleBand::FlexibleBand(const string& name, bool openStructure_) :
      Contour1sFlexible(name), Cn(2, INIT, 0.), width(0.), nDist(0.), openStructure(openStructure_)
#ifdef HAVE_OPENMBVCPPINTERFACE
  , openMBVBody(0), openMBVGrp(0), openMBVNeturalFibre(0)
#endif
  {
  }

  void FlexibleBand::setCn(const Vec& Cn_) {
    assert(Cn_.size() == 2);
    Cn = Cn_ / nrm2(Cn_);
  }

  void FlexibleBand::updateKinematicsForFrame(ContourPointData& cp, FrameFeature ff) {
    if (ff == firstTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy)
      neutral->updateKinematicsForFrame(cp, firstTangent);
    if (ff == normal || ff == secondTangent || ff == cosy || ff == position_cosy || ff == velocity_cosy || ff == velocities_cosy) {
//      static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,normal);
//      static_cast<FlexibleBody*>(parent)->updateKinematicsForFrame(cp,secondTangent);
      neutral->updateKinematicsForFrame(cp, normal);
      neutral->updateKinematicsForFrame(cp, secondTangent);

      Vec WnLocal = cp.getFrameOfReference().getOrientation().col(0);
      Vec WbLocal = cp.getFrameOfReference().getOrientation().col(2);
      if (ff != secondTangent)
        cp.getFrameOfReference().getOrientation().set(0, WnLocal * Cn(0) + WbLocal * Cn(1));
      if (ff != normal)
        cp.getFrameOfReference().getOrientation().set(2, -WnLocal * Cn(1) + WbLocal * Cn(0));
    }
    if (ff == position || ff == position_cosy) {
      neutral->updateKinematicsForFrame(cp, position);
      cp.getFrameOfReference().getPosition() += cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
    }
    if (ff == angularVelocity || ff == velocities || ff == velocities_cosy) {
      neutral->updateKinematicsForFrame(cp, angularVelocity);
    }
    if (ff == velocity || ff == velocity_cosy || ff == velocities || ff == velocities_cosy) {
      neutral->updateKinematicsForFrame(cp, velocity);
      Vec3 dist = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1);
      cp.getFrameOfReference().getVelocity() += crossProduct(cp.getFrameOfReference().getAngularVelocity(), dist);
    }
  }

  void FlexibleBand::updateJacobiansForFrame(ContourPointData &cp, int j /*=0*/) {
    neutral->updateJacobiansForFrame(cp);
    Vec WrPC = cp.getFrameOfReference().getOrientation().col(0) * nDist + cp.getFrameOfReference().getOrientation().col(2) * cp.getLagrangeParameterPosition()(1); // vector from neutral line to contour surface point
    SqrMat tWrPC = tilde(WrPC).copy(); // tilde matrix of above vector
    cp.getFrameOfReference().setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation() - tWrPC * cp.getFrameOfReference().getJacobianOfRotation()); // Jacobian of translation at contour surface with standard description assuming rigid cross-section
  }

  void FlexibleBand::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(openMBV) == enabled && openMBVBody) {

        std::vector<double> plotData;
        plotData.push_back(t);
        Vec X(6, INIT, 0.);

        double length_lagrange = nodes.back() - nodes.front();
        double ds = openStructure ? length_lagrange / (((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints() - 1) : length_lagrange / (((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints() - 2);
        for (int i = 0; i < ((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints(); i++) {

          // Get continous information from neutral fibre
          ContourPointData cp(ds * i);

          openMBVNeturalFibre->updateKinematicsForFrame(cp, MBSim::position);
          openMBVNeturalFibre->updateKinematicsForFrame(cp, MBSim::angle);
          openMBVNeturalFibre->updateKinematicsForFrame(cp, MBSim::normal);
          X(0, 2) = cp.getFrameOfReference().getPosition() + cp.getFrameOfReference().getOrientation().col(0) * nDist;  // the position of the spline curve above the neutral fibre curve
          X(3, 5) = cp.getFrameOfReference().getAnglesOfOrientation();

          Vec pos = R->getPosition() + R->getOrientation() * X(0, 2);
          plotData.push_back(pos(0)); // global x-position
          plotData.push_back(pos(1)); // global y-position
          plotData.push_back(pos(2)); // global z-position
          plotData.push_back(X(3)); // local twist
        }

        ((OpenMBV::SpineExtrusion*) openMBVBody)->append(plotData);
      }
#endif
    }
    Contour::plot(t, dt);
  }

  void FlexibleBand::init(MBSim::InitStage stage) {
    if (stage == MBSim::plot) {
#ifdef HAVE_OPENMBVCPPINTERFACE
//        if(getPlotFeature(openMBV)==enabled && openMBVBody) {
      if (openMBVBody) {
        openMBVBody->setName(name);
        parent->getOpenMBVGrp()->addObject(openMBVBody);
      }
#endif
      Contour::init(stage);
    }
    else
      Contour::init(stage);
  }

//    void FlexibleBand::init(MBSim::InitStage stage){
//    if(stage==MBSim::plot) {
//      if(getPlotFeature(plotRecursive)==enabled) {
//#ifdef HAVE_OPENMBVCPPINTERFACE
//        if (getPlotFeature(openMBV) == enabled && openMBVBody) {
////          openMBVGrp=new OpenMBV::Group();
////          openMBVGrp->setName(name+"_Group");
////          openMBVGrp->setExpand(false);
////          parent->getOpenMBVGrp()->addObject(openMBVGrp);
//          if(getPlotFeature(openMBV)==enabled && openMBVBody) {
//            openMBVBody->setName(name);
//            openMBVGrp->addObject(openMBVBody);
//          }
//        }
//#endif
//        Contour::init(stage);
//      }
//    }else
//      Contour::init(stage);
//  }

}

