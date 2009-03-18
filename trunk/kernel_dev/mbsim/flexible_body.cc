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
#include <mbsim/contour_pdata.h>
#include <mbsim/frame.h>

#ifdef HAVE_AMVIS
#include "elastic.h"
using namespace AMVis;
#endif

namespace MBSim {

  FlexibleBody::FlexibleBody(const string &name) : Body(name), frameParent(0), d_massproportional(0.)
# ifdef HAVE_AMVIS
                                                   , bodyAMVis(NULL), boolAMVis(false), boolAMVisBinary(true), AMVisColor(0.)
# endif
                                                   {}

  FlexibleBody::~FlexibleBody() {
    for(unsigned int i=0; i<discretization.size(); i++) {
      delete discretization[i]; discretization[i] = NULL;
    }
#  ifdef HAVE_AMVIS
    delete bodyAMVis; bodyAMVis = NULL;
#  endif
  }

  void FlexibleBody::updateh(double t) {
    M.init(0.);
    h.init(0.);

    for(unsigned int i=0;i<discretization.size();i++) {
      discretization[i]->computeEquationsOfMotion(qElement[i],uElement[i]); // compute attributes of finite element

      GlobalMatrixContribution(i); // assemble
    }

    if(d_massproportional) h -= d_massproportional*(M*u); // mass proportional damping
  }

  void FlexibleBody::updateKinematics(double t) {
    for(unsigned int i=0; i<port.size(); i++) { // frames
      port[i]->setPosition(computeWrOC(S_Frame[i])); 
      port[i]->setOrientation(computeAWK(S_Frame[i]));
      port[i]->setVelocity(computeWvC(S_Frame[i]));
      port[i]->setAngularVelocity(computeWomega(S_Frame[i]));
    }
    // TODO contour non native?
  }

  void FlexibleBody::updateJacobians(double t) {
    for(unsigned int i=0; i<port.size(); i++) { // frames
      Mat Jactmp = computeJacobianMatrix(S_Frame[i]); // temporial Jacobian matrix
      port[i]->setJacobianOfTranslation(Jactmp*trans(JT)); // TODO sparse structure / port in frame
      port[i]->setJacobianOfRotation(Jactmp*trans(JR));
      // port[i]->setGyroscopicAccelerationOfTranslation(); TODO
      // port[i]->setGyroscopicAccelerationOfRotation(); TODO
    }
    // TODO contour non native?
  }

  void FlexibleBody::updateSecondJacobians(double t) {}

  void FlexibleBody::plot(double t, double dt, bool top) {
    if(getPlotFeature(plotRecursive)==enabled) {
      Element::plot(t,dt,false); // only time

      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<qSize; ++i) plotVector.push_back(q(i));
        for(int i=0; i<uSize[0]; ++i) plotVector.push_back(u(i));
        double Ttemp = computeKineticEnergy();
        double Vtemp = computePotentialEnergy();
        plotVector.push_back(Ttemp);
        plotVector.push_back(Vtemp);
        plotVector.push_back(Ttemp + Vtemp);
      }
    }

#ifdef HAVE_AMVIS
    if(boolAMVis) {
      float *qDummy = (float*) malloc(qSize*sizeof(float));
      for(int i=0;i<qSize;i++) qDummy[i] = q(i);
      bodyAMVis->setTime(t);
      bodyAMVis->setCoordinates(qDummy);
      bodyAMVis->appendDataset(0);
      free(qDummy);
    }
#endif
  }

  void FlexibleBody::initPlot(bool top) {
    Element::initPlot(parent, true, false); // only time

    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
        for(int i=0; i<qSize; ++i) { stringstream index; index << "q(" << i << ")"; plotColumns.push_back(index.str()); }
        for(int i=0; i<uSize[0]; ++i) { stringstream index; index << "u(" << i << ")"; plotColumns.push_back(index.str()); }
        plotColumns.push_back("T");
        plotColumns.push_back("V");
        plotColumns.push_back("E");
      }
    }

    if(top) createDefaultPlot();

#ifdef HAVE_AMVIS
    if(boolAMVis && getPlotFeature(amvis)==enabled)
      bodyAMVis->writeBodyFile();
#endif
  }

  void FlexibleBody::init() {
    Body::init();
    T = SqrMat(qSize,fmatvec::EYE);
  }

  double FlexibleBody::computePotentialEnergy() {
    double V = 0.;
    for(unsigned int i=0; i<discretization.size(); i++) {
      V += discretization[i]->computeElasticEnergy(qElement[i]) + discretization[i]->computeGravitationalEnergy(qElement[i]);
    }
    return V;
  }

  void FlexibleBody::addFrame(const string &name, const ContourPointData &S_) {
    Frame *port = new Frame(name);
    addFrame(port,S_);
  }

  void FlexibleBody::addFrame(Frame* port, const ContourPointData &S_) {
    Body::addFrame(port);
    S_Frame.push_back(S_);
  }

  FlexibleBody1s::FlexibleBody1s(const string &name) : FlexibleBody(name), userContourNodes(0) {}

  void FlexibleBody1s::addFrame(const string &name, const double &s) {
    ContourPointData temp;
    temp.type  = CONTINUUM;
    temp.alpha = Vec(1,INIT,s);
    FlexibleBody::addFrame(name,temp); 
  }

  Vec FlexibleBody1s::computeWrOC(const double &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = Vec(1,INIT,alpha); 
    return FlexibleBody::computeWrOC(temp); 
  }

  SqrMat FlexibleBody1s::computeAWK(const double &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = Vec(1,INIT,alpha); 
    return FlexibleBody::computeAWK(temp); 
  }

  Vec FlexibleBody1s::computeWvC(const double &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = Vec(1,INIT,alpha); 
    return FlexibleBody::computeWvC(temp); 
  }

  Vec FlexibleBody1s::computeWomega(const double &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = Vec(1,INIT,alpha); 
    return FlexibleBody::computeWomega(temp); 
  }

  FlexibleBody2s::FlexibleBody2s(const string &name) :FlexibleBody(name) {}

  void FlexibleBody2s::addFrame(const string &name, const Vec &alpha) {
    ContourPointData temp;
    temp.type  = CONTINUUM;
    temp.alpha = alpha;
    FlexibleBody::addFrame(name,temp); 
  }

  Vec FlexibleBody2s::computeWrOC(const Vec &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = alpha; 
    return FlexibleBody::computeWrOC(temp); 
  }

  SqrMat FlexibleBody2s::computeAWK(const Vec &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM; 
    temp.alpha = alpha; 
    return FlexibleBody::computeAWK(temp); 
  }

  Vec FlexibleBody2s::computeWvC(const Vec &alpha) {
    ContourPointData temp;
    temp.type = CONTINUUM;
    temp.alpha = alpha;
    return FlexibleBody::computeWvC(temp); 
  }

  Vec FlexibleBody2s::computeWomega(const Vec &alpha) { 
    ContourPointData temp; 
    temp.type = CONTINUUM;
    temp.alpha = alpha; 
    return FlexibleBody::computeWomega(temp);
  }
}

