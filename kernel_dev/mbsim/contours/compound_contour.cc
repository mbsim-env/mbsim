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

using namespace std;
using namespace fmatvec;

namespace MBSim {

  CompoundContour::CompoundContour(const string &name) : Contour(name) {
  }

  void CompoundContour::addContourElement(RigidContour* c, const Vec& Kr_) {
    element.push_back(c);
    Kr.push_back(Kr_);
    Wr.push_back(Vec(3));
  }

  void CompoundContour::setReferencePosition(const Vec &WrOP) {
    Contour::setReferencePosition(WrOP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferencePosition(R.getPosition() + Wr[i]);
  }

  void CompoundContour::setReferenceVelocity(const Vec &WvP) {
    Contour::setReferenceVelocity(WvP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceVelocity(R.getVelocity() + crossProduct(R.getAngularVelocity(), Wr[i]));
  }

  void CompoundContour::setReferenceAngularVelocity(const Vec &WomegaC) {
    Contour::setReferenceAngularVelocity(WomegaC);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceAngularVelocity(R.getAngularVelocity());
  }

  void CompoundContour::setReferenceOrientation(const SqrMat &AWC) {
    Contour::setReferenceOrientation(AWC);
    for(unsigned int i=0; i<element.size(); i++) {
      element[i]->setReferenceOrientation(R.getOrientation());
      Wr[i] = R.getOrientation()*Kr[i];
    }
  }

  void CompoundContour::setReferenceJacobianOfTranslation(const Mat &WJP) {
    Contour::setReferenceJacobianOfTranslation(WJP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceJacobianOfTranslation(R.getJacobianOfTranslation() - tilde(Wr[i])*R.getJacobianOfRotation());
  }

  void CompoundContour::setReferenceGyroscopicAccelerationOfTranslation(const Vec &WjP) {
    Contour::setReferenceGyroscopicAccelerationOfTranslation(WjP);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceGyroscopicAccelerationOfTranslation(R.getGyroscopicAccelerationOfTranslation() - tilde(Wr[i])*R.getGyroscopicAccelerationOfRotation() + crossProduct(R.getAngularVelocity(),crossProduct(R.getAngularVelocity(),Wr[i])));
  }

  void CompoundContour::setReferenceJacobianOfRotation(const Mat &WJR) {
    Contour::setReferenceJacobianOfRotation(WJR);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceJacobianOfRotation(R.getJacobianOfRotation());
  }

  void CompoundContour::setReferenceGyroscopicAccelerationOfRotation(const Vec &WjR) {
    Contour::setReferenceGyroscopicAccelerationOfRotation(WjR);
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->setReferenceGyroscopicAccelerationOfRotation(R.getGyroscopicAccelerationOfRotation());
  }

  void CompoundContour::init(InitStage stage) {
    if(stage==unknownStage) {
      Contour::init(stage);
      for(unsigned int i=0; i<element.size(); i++)
        element[i]->sethSize(hSize[0]);
    }
    else
      Contour::init(stage);

    for(unsigned int i=0; i<element.size(); i++) {
      element[i]->setParent(parent);
      element[i]->init(stage);
    }
  }

  void CompoundContour::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->updateKinematicsForFrame(cp,ff);
  }

  void CompoundContour::updateJacobiansForFrame(ContourPointData &cp) {
    for(unsigned int i=0; i<element.size(); i++) 
      element[i]->updateJacobiansForFrame(cp);
  }

}
