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
#include <mbsim/subsystem.h>
#include <mbsim/frame.h>
#include <mbsim/utils/function.h>
#include <mbsim/mbsim_event.h>

#ifdef HAVE_AMVIS
#include "elastic.h"
using namespace AMVis;
#endif

namespace MBSim {

  template <class AT> FlexibleBody<AT>::FlexibleBody(const string &name) : Body(name), frameParent(0), d_massproportional(0.)
# ifdef HAVE_AMVIS
                                                                           , boolAMVis(false), bodyAMVis(NULL), boolAMVisBinary(true), AMVisColor(0.)
# endif
                                                                           {}

  template <class AT> FlexibleBody<AT>::~FlexibleBody() {
    for(unsigned int i=0; i<discretization.size(); i++) {
      delete discretization[i]; discretization[i] = NULL;
    }
#  ifdef HAVE_AMVIS
    delete bodyAMVis; bodyAMVis = NULL;
#  endif
  }

  template <class AT> void FlexibleBody<AT>::updateM(double t) {
    for(unsigned int i=0;i<discretization.size();i++) {
      discretization[i]->computeEquationsOfMotion(qElement[i],uElement[i]); // compute attributes of finite element

      GlobalMatrixContribution(i); // assemble
    }

    if(d_massproportional) h -= d_massproportional*(M*u); // mass proportional damping
  }

  template <class AT> void FlexibleBody<AT>::updateKinematics(double t) {
    BuildElements();
    for(unsigned int i=0; i<port.size(); i++) { // frames
      updateKinematicsForFrame(S_Frame[i],port[i]); 
    }
    // TODO contour non native?
  }

  template <class AT> void FlexibleBody<AT>::updateJacobians(double t) {
    for(unsigned int i=0; i<port.size(); i++) { // frames
      updateJacobiansForFrame(S_Frame[i],port[i]);
    }
    // TODO contour non native?
  }

  template <class AT> void FlexibleBody<AT>::updateSecondJacobians(double t) {
    throw new MBSimError("ERROR (FlexibleBody::updateSecondJacobians): Not implemented!");
  }

  template <class AT> void FlexibleBody<AT>::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVIS
      if(bodyAMVis && getPlotFeature(amvis)==enabled) {
        float *qDummy = (float*) malloc(qSize*sizeof(float));
        for(int i=0;i<qSize;i++) qDummy[i] = q(i);
        bodyAMVis->setTime(t);
        bodyAMVis->setCoordinates(qDummy);
        bodyAMVis->appendDataset(0);
        free(qDummy);
      }
#endif
      Body::plot(t,dt);
    }
  }

  template <class AT> void FlexibleBody<AT>::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVIS
      if(bodyAMVis && getPlotFeature(amvis)==enabled)
        bodyAMVis->writeBodyFile();
#endif

      Body::initPlot();
    }
  }

  template <class AT> void FlexibleBody<AT>::init() {
    Body::init();
    T = SqrMat(qSize,fmatvec::EYE);
    for(unsigned int i=0; i<port.size(); i++) { // frames
      S_Frame[i].cosy.getJacobianOfTranslation().resize(3,uSize[0]);
      S_Frame[i].cosy.getJacobianOfRotation().resize(3,uSize[0]);
    }
  }

  template <class AT> double FlexibleBody<AT>::computeKineticEnergy() {
    double T = 0.;
    for(unsigned int i=0; i<discretization.size(); i++) {
      T += discretization[i]->computeKineticEnergy(qElement[i],uElement[i]);
    }
    return T;
  }

  template <class AT> double FlexibleBody<AT>::computePotentialEnergy() {
    double V = 0.;
    for(unsigned int i=0; i<discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(qElement[i]) + discretization[i]->computeGravitationalEnergy(qElement[i]);
    }
    return V;
  }

  template <class AT> void FlexibleBody<AT>::addFrame(const string &name, const ContourPointData &S_) {
    Frame *port = new Frame(name);
    addFrame(port,S_);
  }

  template <class AT> void FlexibleBody<AT>::addFrame(Frame* port, const ContourPointData &S_) {
    Body::addFrame(port);
    S_Frame.push_back(S_);
  }

  template class FlexibleBody<Vec>;
  template class FlexibleBody<Mat>;
}

