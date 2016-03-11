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
 *          martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <mbsimFlexibleBody/flexible_body.h>
#include <mbsimFlexibleBody/frames/node_frame.h>
#include <mbsim/dynamic_system.h>
#include <mbsim/frames/fixed_relative_frame.h>
#include <mbsim/contours/contour.h>
#include <fmatvec/function.h>
#include <mbsim/mbsim_event.h>
#include <mbsimFlexibleBody/discretization_interface.h>

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  FlexibleBody::FlexibleBody(const string &name) : Body(name), d_massproportional(0.), updEle(true) { }

  FlexibleBody::~FlexibleBody() {
    for (unsigned int i = 0; i < discretization.size(); i++) {
      if (discretization[i]) {
        delete discretization[i];
        discretization[i] = NULL;
      }
    }
  }

  void FlexibleBody::updateh(double t, int k) {
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeh(getqElement(t,i), getuElement(t,i)); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalVectorContribution(i, discretization[i]->geth(), h[k]); // assemble

    if (d_massproportional > 0) { // mass proportional damping
      h[k] -= d_massproportional * (M[k] * u);
    }
  }

  void FlexibleBody::updateM(double t, int k) {
    for (int i = 0; i < (int) discretization.size(); i++)
      discretization[i]->computeM(getqElement(t,i)); // compute attributes of finite element
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

  void FlexibleBody::updatePositions(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updatePositions): Not implemented.");
  }

  void FlexibleBody::updateVelocities(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateVelocities): Not implemented.");
  }

  void FlexibleBody::updateAccelerations(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateAccelerations): Not implemented.");
  }

  void FlexibleBody::updateJacobians(double t, NodeFrame* frame, int j) {
    THROW_MBSIMERROR("(FlexibleBody::updateJacobians): Not implemented.");
  }

  void FlexibleBody::updateGyroscopicAccelerations(double t, NodeFrame* frame) {
    THROW_MBSIMERROR("(FlexibleBody::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
      Body::plot(t, dt);
    }
  }

  void FlexibleBody::init(InitStage stage) {
    if (stage == resize) {
      Body::init(stage);
      for(vector<Frame*>::iterator i=nonUserFrame.begin(); i!=nonUserFrame.end(); i++) {
        (*i)->sethSize(hSize[0],0);
        (*i)->sethInd(hInd[0],0);
        (*i)->sethSize(hSize[1],1);
        (*i)->sethInd(hInd[1],1);
      }
    }
    else if (stage == unknownStage) {
      Body::init(stage);
      T = SqrMat(qSize, EYE);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
        Body::init(stage);
      }
    }
    else
      Body::init(stage);

    for(vector<Frame*>::iterator i=nonUserFrame.begin(); i!=nonUserFrame.end(); i++)
      (*i)->init(stage);
  }

  double FlexibleBody::computeKineticEnergy(double t) {
    double T = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      T += discretization[i]->computeKineticEnergy(getqElement(t,i), getuElement(t,i));
    }
    return T;
  }

  double FlexibleBody::computePotentialEnergy(double t) {
    double V = 0.;
    for (unsigned int i = 0; i < discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(getqElement(t,i)) + discretization[i]->computeGravitationalEnergy(getqElement(t,i));
    }
    return V;
  }

  void FlexibleBody::setFrameOfReference(Frame *frame) {
    if (dynamic_cast<DynamicSystem*>(frame->getParent()))
      R = frame;
    else
      THROW_MBSIMERROR("(FlexibleBody::setFrameOfReference): Only stationary reference frames are implemented at the moment!");
  }

  void FlexibleBody::addFrame(NodeFrame *frame) {
    Body::addFrame(frame);
  }

  void FlexibleBody::addFrame(FixedRelativeFrame *frame) {
    Body::addFrame(frame);
  }

  void FlexibleBody::addContour(Contour *contour_) {
    Body::addContour(contour_);
  }

  void FlexibleBody::addNonUserFrame(Frame* frame) {
    nonUserFrame.push_back(frame);
    frame->setParent(this);
  }

  void FlexibleBody::initializeUsingXML(DOMElement *element) {
    Body::initializeUsingXML(element);
    
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"massProportionalDamping");
    setMassProportionalDamping(getDouble(e));
  }

  void FlexibleBody::resetUpToDate() {
    Body::resetUpToDate();
    for(unsigned int i=0; i<nonUserFrame.size(); i++)
      nonUserFrame[i]->resetUpToDate();
    updEle = true;
  }

}
