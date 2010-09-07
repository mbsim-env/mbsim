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
 *          rzander@users.berlios.de
 */

#include <config.h>
#include <mbsim/flexible_body.h>
#include <mbsim/dynamic_system.h>
#include <mbsim/frame.h>
#include <mbsim/utils/function.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/discretization_interface.h>

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  FlexibleBody::FlexibleBody(const string &name) : Body(name), d_massproportional(0.) {}

  FlexibleBody::~FlexibleBody() {
    for(unsigned int i=0; i<discretization.size(); i++) {
      delete discretization[i]; discretization[i] = NULL;
    }
  }

  void FlexibleBody::updateh(double t) {
//#pragma omp parallel for schedule(static) shared(t) default(none) if((int)discretization.size()>4) 
    for(int i=0;i<(int)discretization.size();i++) {
      try { discretization[i]->computeh(qElement[i],uElement[i]); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)discretization.size();i++) GlobalVectorContribution(i,discretization[i]->geth(),h); // assemble
    for(int i=0;i<(int)discretization.size();i++) GlobalVectorContribution(i,discretization[i]->geth(),hObject); // assemble

    if(d_massproportional>0) { // mass proportional damping
      h -= d_massproportional*(M*u);
      hObject -= d_massproportional*(M*u);
    }
  }

  void FlexibleBody::updateM(double t) {
//#pragma omp parallel for schedule(static) shared(t) default(none) if((int)discretization.size()>4) 
    for(int i=0;i<(int)discretization.size();i++) {
      try { discretization[i]->computeM(qElement[i]); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)discretization.size();i++) GlobalMatrixContribution(i,discretization[i]->getM(),M); // assemble
  }

  void FlexibleBody::updatedhdz(double t) {
    updateh(t);
//#pragma omp parallel for schedule(static) shared(t) default(none) if((int)discretization.size()>4) 
    for(int i=0;i<(int)discretization.size();i++) {
      try {discretization[i]->computedhdz(qElement[i],uElement[i]); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
   for(int i=0;i<(int)discretization.size();i++) GlobalMatrixContribution(i,discretization[i]->getdhdq(),dhdq); // assemble
   for(int i=0;i<(int)discretization.size();i++) GlobalMatrixContribution(i,discretization[i]->getdhdu(),dhdu); // assemble
  }

  void FlexibleBody::updateStateDependentVariables(double t) {
    BuildElements();
    for(unsigned int i=0; i<frame.size(); i++) { // frames
      updateKinematicsForFrame(S_Frame[i],all,frame[i]); 
    }
    // TODO contour non native?
  }

  void FlexibleBody::updateJacobians(double t) {
    for(unsigned int i=0; i<frame.size(); i++) { // frames
      updateJacobiansForFrame(S_Frame[i],frame[i]);
    }
    // TODO contour non native?
  }

  void FlexibleBody::updateInverseKineticsJacobians(double t) {
    throw MBSimError("ERROR (FlexibleBody::updateInverseKineticsJacobians): Not implemented!");
  }

  void FlexibleBody::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Body::plot(t,dt);
    }
  }

  void FlexibleBody::init(InitStage stage) {
    if(stage==unknownStage) {
      Body::init(stage);
      T = SqrMat(qSize,fmatvec::EYE);
      for(unsigned int i=0; i<frame.size(); i++) { // frames
        S_Frame[i].getFrameOfReference().getJacobianOfTranslation().resize(3,uSize[0]);
        S_Frame[i].getFrameOfReference().getJacobianOfRotation().resize(3,uSize[0]);
      }
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
        Body::init(stage);
      }
    }
    else
      Body::init(stage);
  }

  double FlexibleBody::computeKineticEnergy() {
    double T = 0.;
    for(unsigned int i=0; i<discretization.size(); i++) {
      T += discretization[i]->computeKineticEnergy(qElement[i],uElement[i]);
    }
    return T;
  }

  double FlexibleBody::computePotentialEnergy() {
    double V = 0.;
    for(unsigned int i=0; i<discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(qElement[i]) + discretization[i]->computeGravitationalEnergy(qElement[i]);
    }
    return V;
  }

  void FlexibleBody::setFrameOfReference(Frame *frame) { 
    if(dynamic_cast<DynamicSystem*>(frame->getParent())) 
      frameOfReference = frame; 
    else 
      throw MBSimError("ERROR (FlexibleBody::setFrameOfReference): Only stationary reference frames are implemented at the moment!"); 
  }

  void FlexibleBody::addFrame(const string &name, const ContourPointData &S_) {
    Frame *frame = new Frame(name);
    addFrame(frame,S_);
  }

  void FlexibleBody::addFrame(Frame* frame, const ContourPointData &S_) {
    Body::addFrame(frame);
    S_Frame.push_back(S_);
  }

  void FlexibleBody::addFrame(const std::string &name, const int &id) {
    ContourPointData cp(id);
    addFrame(name,cp);
  }

  void FlexibleBody::addFrame(Frame *frame, const  int &id) {
    ContourPointData cp(id);
    addFrame(frame,cp);
  }

}

