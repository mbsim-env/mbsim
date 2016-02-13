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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#include <config.h>
#include <mbsimFlexibleBody/flexible_body.h>
#include <mbsimFlexibleBody/node_frame.h>
#include <mbsimFlexibleBody/contour_parameter_frame.h>
#include <mbsim/dynamic_system.h>
#include <mbsim/fixed_relative_frame.h>
#include <mbsim/contour.h>
#include <fmatvec/function.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/discretization_interface.h>


//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  FlexibleBody::FlexibleBody(const string &name) : Body(name), d_massproportional(0.), updEle(true) {
//    contourFrame = new Frame("ContourFrame");
//    contourFrame->setParent(this);
    //addFrame(contourFrame,0);
  }

  FlexibleBody::~FlexibleBody() {
    for (unsigned int i = 0; i < discretization.size(); i++) {
      if (discretization[i]) {
        delete discretization[i];
        discretization[i] = NULL;
      }
    }
//    delete contourFrame;
  }

  void FlexibleBody::updateh(double t, int k) {
    if(updEle) BuildElements();
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeh(qElement[i], uElement[i]); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalVectorContribution(i, discretization[i]->geth(), h[k]); // assemble

    if (d_massproportional > 0) { // mass proportional damping
      h[k] -= d_massproportional * (M[k] * u);
    }
  }

  void FlexibleBody::updateM(double t, int k) {
    if(updEle) BuildElements();
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeM(qElement[i]); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), M[k]); // assemble
  }

  void FlexibleBody::updatedhdz(double t) {
    updateh(t);
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computedhdz(qElement[i], uElement[i]); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getdhdq(), dhdq); // assemble
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getdhdu(), dhdu); // assemble
  }

  void FlexibleBody::updatePositions(double t, ContourFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updatePositions): Not implemented.");
  }

  void FlexibleBody::updatePositions(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updatePositions): Not implemented.");
  }

  void FlexibleBody::updateVelocities(double t, ContourFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateVelocities): Not implemented.");
  }

  void FlexibleBody::updateVelocities(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateVelocities): Not implemented.");
  }

  void FlexibleBody::updateAccelerations(double t, ContourFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateAccelerations): Not implemented.");
  }

  void FlexibleBody::updateAccelerations(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateAccelerations): Not implemented.");
  }

  void FlexibleBody::updateJacobians(double t, ContourFrame* frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody::updateJacobians): Not implemented.");
  }

  void FlexibleBody::updateJacobians(double t, NodeFrame* frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody::updateJacobians): Not implemented.");
  }

  void FlexibleBody::updateGyroscopicAccelerations(double t, ContourFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody::updateGyroscopicAccelerations(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateGyroscopicAccelerations): Not implemented.");
  }

  Vec3 FlexibleBody::getWu(double t, const Vec2 &zeta) {
    THROW_MBSIMERROR("(FlexibleBody::getPosition): Not implemented.");
    return 0;
  }

  Vec3 FlexibleBody::getWu(double t, int node) {
    THROW_MBSIMERROR("(FlexibleBody::getPosition): Not implemented.");
    return 0;
  }

  void FlexibleBody::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
      Body::plot(t, dt);
    }
  }

  void FlexibleBody::init(InitStage stage) {
    if (stage == unknownStage) {
      Body::init(stage);
      T = SqrMat(qSize, EYE);
//      for (unsigned int i = 0; i < S_Frame.size(); i++) { // frames
//        S_Frame[i].getFrameOfReference().getJacobianOfTranslation(0,false).resize(uSize[0]);
//        S_Frame[i].getFrameOfReference().getJacobianOfRotation(0,false).resize(uSize[0]);
//      }
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
        Body::init(stage);
      }
    }
    else
      Body::init(stage);
  }

  double FlexibleBody::computeKineticEnergy() {
    double T = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      T += discretization[i]->computeKineticEnergy(qElement[i], uElement[i]);
    }
    return T;
  }

  double FlexibleBody::computePotentialEnergy() {
    double V = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(qElement[i]) + discretization[i]->computeGravitationalEnergy(qElement[i]);
    }
    return V;
  }

  void FlexibleBody::setFrameOfReference(Frame *frame) {
    if (dynamic_cast<DynamicSystem*>(frame->getParent()))
      R = frame;
    else
      THROW_MBSIMERROR("(FlexibleBody::setFrameOfReference): Only stationary reference frames are implemented at the moment!");
  }

//  void FlexibleBody::addFrame(const string &name, const ContourPointData &S_) {
//    Frame *frame = new Frame(name);
//    addFrame(frame, S_);
//  }

//  void FlexibleBody::addFrame(Frame* frame, const ContourPointData &S_) {
//    Body::addFrame(frame);
//    S_Frame.push_back(S_);
//  }

  void FlexibleBody::addFrame(NodeFrame *frame) {
    Body::addFrame(frame);
  }

  void FlexibleBody::addFrame(FixedContourFrame *frame) {
    Body::addFrame(frame);
  }

  void FlexibleBody::addFrame(MBSim::FixedRelativeFrame *frame) {
    Body::addFrame(frame);
  }

  void FlexibleBody::addContour(Contour *contour_) {
//    contour_->setFrameOfReference(contourFrame);
    Body::addContour(contour_);
  }

  void FlexibleBody::initializeUsingXML(DOMElement *element) {
    Body::initializeUsingXML(element);
    
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"massProportionalDamping");
    setMassProportionalDamping(getDouble(e));
  }

  void FlexibleBody::resetUpToDate() {
    Body::resetUpToDate();
    updEle = true;
  }

}

