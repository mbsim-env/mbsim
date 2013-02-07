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

#include<config.h>
#include "mbsim/contours/compound_contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  CompoundContour::CompoundContour(const string &name) :
      RigidContour(name),
#ifdef HAVE_OPENMBVCPPINTERFACE
          openMBVGroup(0)
#endif
  {
  }

  void CompoundContour::addContourElement(Contour* c, const Vec3& Kr_, const SqrMat3& AIK_) {
    element.push_back(c);
    c->setParent(this);
    Kr.push_back(Kr_);
    Wr.push_back(Vec3());
    AIK.push_back(AIK_);
  }

  void CompoundContour::plot(double t, double dt) {
    for(size_t i =0; i < element.size(); i++) {
      element[i]->plot(t, dt);
    }
  }

  void CompoundContour::setReferencePosition(const Vec3 &WrOP) {
    Contour::setReferencePosition(WrOP);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferencePosition(R.getPosition() + Wr[i]);
  }

  void CompoundContour::setReferenceVelocity(const Vec3 &WvP) {
    Contour::setReferenceVelocity(WvP);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceVelocity(R.getVelocity() + crossProduct(R.getAngularVelocity(), Wr[i]));
  }

  void CompoundContour::setReferenceAngularVelocity(const Vec3 &WomegaC) {
    Contour::setReferenceAngularVelocity(WomegaC);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceAngularVelocity(R.getAngularVelocity());
  }

  void CompoundContour::setReferenceOrientation(const SqrMat3 &AWC) {
    Contour::setReferenceOrientation(AWC);
    for (unsigned int i = 0; i < element.size(); i++) {
      element[i]->setReferenceOrientation(R.getOrientation() * AIK[i]);
      Wr[i] = R.getOrientation() * Kr[i];
    }
  }

  void CompoundContour::setReferenceJacobianOfTranslation(const Mat3V &WJP) {
    Contour::setReferenceJacobianOfTranslation(WJP);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceJacobianOfTranslation(R.getJacobianOfTranslation() - tilde(Wr[i]) * R.getJacobianOfRotation());
  }

  void CompoundContour::setReferenceGyroscopicAccelerationOfTranslation(const Vec3 &WjP) {
    Contour::setReferenceGyroscopicAccelerationOfTranslation(WjP);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceGyroscopicAccelerationOfTranslation(R.getGyroscopicAccelerationOfTranslation() - tilde(Wr[i]) * R.getGyroscopicAccelerationOfRotation() + crossProduct(R.getAngularVelocity(), crossProduct(R.getAngularVelocity(), Wr[i])));
  }

  void CompoundContour::setReferenceJacobianOfRotation(const Mat3V &WJR) {
    Contour::setReferenceJacobianOfRotation(WJR);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceJacobianOfRotation(R.getJacobianOfRotation());
  }

  void CompoundContour::setReferenceGyroscopicAccelerationOfRotation(const Vec3 &WjR) {
    Contour::setReferenceGyroscopicAccelerationOfRotation(WjR);
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->setReferenceGyroscopicAccelerationOfRotation(R.getGyroscopicAccelerationOfRotation());
  }

  void CompoundContour::init(InitStage stage) {
    if (stage == unknownStage) {
      Contour::init(stage);
      for (unsigned int i = 0; i < element.size(); i++)
        element[i]->sethSize(hSize[0]);
    }
    else if (stage == MBSim::plot) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (parent)
        updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
        if (openMBVGroup == 0) {
          openMBVGroup = new OpenMBV::Group();
          openMBVGroup->setName(name + "Group");
          //if(parent) parent->openMBVGrp->addObject(openMBVGrp);
          if (parent)
            parent->getOpenMBVGrp()->addObject(openMBVGroup);
          if (getPlotFeature(separateFilePerGroup) == enabled)
            openMBVGroup->setSeparateFile(true);
        }
#endif
      }
    }
    Contour::init(stage);

    for (unsigned int i = 0; i < element.size(); i++) {
      //element[i]->setParent(parent); // PARENT
      element[i]->init(stage);
    }
  }

  void CompoundContour::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->updateKinematicsForFrame(cp, ff);
  }

  void CompoundContour::updateJacobiansForFrame(ContourPointData &cp) {
    for (unsigned int i = 0; i < element.size(); i++)
      element[i]->updateJacobiansForFrame(cp);
  }

}
