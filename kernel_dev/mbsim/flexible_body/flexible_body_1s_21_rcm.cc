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
 * Contact: rzander@users.berlios.de
 *          thschindler@users.berlios.de
 */

#include <config.h>
#include <mbsim/flexible_body/flexible_body_1s_21_rcm.h>
#include <mbsim/flexible_body/finite_elements/finite_element_1s_21_rcm.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/contour.h>

#define FMATVEC_DEEP_COPY

#ifdef HAVE_AMVIS
#include "elastic1s21rcm.h"
using namespace AMVis;
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  FlexibleBody1s21RCM::FlexibleBody1s21RCM(const string &name, bool openStructure_) : FlexibleBody<Vec>(name), L(0), l0(0), E(0), A(0), I(0), rho(0), rc(0), dm(0), dl(0), openStructure(openStructure_), initialized(false)
#ifdef HAVE_AMVIS
                                                                                     ,
                                                                                     AMVisRadius(0), AMVisBreadth(0), AMVisHeight(0)
#endif

                                                                                     { 
                                                                                       contourR = new Contour1sFlexible("R");
                                                                                       contourL = new Contour1sFlexible("L");
                                                                                       Body::addContour(contourR);
                                                                                       Body::addContour(contourL);
                                                                                     }

  void FlexibleBody1s21RCM::BuildElements() {
    for(int i=0;i<Elements;i++) {

      int n = 5 * i ;

      if(i<Elements-1 || openStructure==true) {
        qElement[i] << q(n,n+7);
        uElement[i] << u(n,n+7);
      }
      else { // last finite element and ring closure
        qElement[i](0,4) << q(n,n+4);
        uElement[i](0,4) << u(n,n+4);
        qElement[i](5,7) << q(0,2);
        if(qElement[i](2)-q(2)>0.0) 
          qElement[i](7) += 2*M_PI;
        else
          qElement[i](7) -= 2*M_PI;
        uElement[i](5,7) << u(0,2);
      } 
    }
  }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n) {
    int j = 5 * n;

    if( n < Elements - 1 || openStructure==true) {
      M(Index(j,j+7))   += discretization[n]->getMassMatrix();
      h(j,j+7)          += discretization[n]->getGeneralizedForceVector();
    } 
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      M(Index(j,j+4))               += discretization[n]->getMassMatrix()(Index(0,4));
      M(Index(j,j+4),Index(0,2))    += discretization[n]->getMassMatrix()(Index(0,4),Index(5,7));
      M(Index(0,2))                 += discretization[n]->getMassMatrix()(Index(5,7));

      h(j,j+4)                      += discretization[n]->getGeneralizedForceVector()(0,4);
      h(0,  2)                      += discretization[n]->getGeneralizedForceVector()(5,7);
    }
  }

  void FlexibleBody1s21RCM::updateKinematicsForFrame(ContourPointData &cp, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      const double &s = cp.getLagrangeParameterPosition()(0); 
      double sLocal = BuildElement(s);
      Vec Z = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->StateBeam(qElement[CurrentElement],uElement[CurrentElement],sLocal);

      Vec tmp(3,NONINIT); tmp(0) = Z(0); tmp(1) = Z(1); tmp(2) = 0.; // temporary vector used for compensating planar description
      cp.getFrameOfReference().setPosition(frameParent->getPosition() + frameParent->getOrientation() * tmp);

      tmp(0) = cos(Z(2)); tmp(1) = sin(Z(2)); 
      cp.getFrameOfReference().getOrientation().col(1) = frameParent->getOrientation() * tmp; // tangent

      tmp(0) = -sin(Z(2)); tmp(1) = cos(Z(2));
      cp.getFrameOfReference().getOrientation().col(0) = frameParent->getOrientation() * tmp; // normal
      cp.getFrameOfReference().getOrientation().col(2) = -frameParent->getOrientation().col(2); // binormal (cartesian system)

      tmp(0) = Z(3); tmp(1) = Z(4);
      cp.getFrameOfReference().setVelocity(frameParent->getOrientation() * tmp);

      tmp(0) = 0.; tmp(1) = 0.; tmp(2) = Z(5);
      cp.getFrameOfReference().setAngularVelocity(frameParent->getOrientation() * tmp);
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT); tmp(0) = q(5*node+0); tmp(1) = q(5*node+1); tmp(2) = 0.; // temporary vector used for compensating planar description
      cp.getFrameOfReference().setPosition(frameParent->getPosition() + frameParent->getOrientation() * tmp);

      tmp(0) =  cos(q(5*node+2)); tmp(1) = sin(q(5*node+2)); 
      cp.getFrameOfReference().getOrientation().col(1)    = frameParent->getOrientation() * tmp; // tangent

      tmp(0) = -sin(q(5*node+2)); tmp(1) = cos(q(5*node+2));
      cp.getFrameOfReference().getOrientation().col(0)    =  frameParent->getOrientation() * tmp; // normal
      cp.getFrameOfReference().getOrientation().col(2)    = -frameParent->getOrientation().col(2); // binormal (cartesian system)

      tmp(0) = u(5*node+0); tmp(1) = u(5*node+1);
      cp.getFrameOfReference().setVelocity(frameParent->getOrientation() * tmp);

      tmp(0) = 0.; tmp(1) = 0.; tmp(2) = u(5*node+2);
      cp.getFrameOfReference().setAngularVelocity(frameParent->getOrientation() * tmp);
    }

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s21RCM::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,3-1);
    Mat Jacobian(qSize,3,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double s = cp.getLagrangeParameterPosition()(0); 
      double sLocal = BuildElement(s);
      Mat Jtmp = dynamic_cast<FiniteElement1s21RCM*>(discretization[CurrentElement])->JGeneralized(qElement[CurrentElement],sLocal);
      if(CurrentElement<Elements-1 || openStructure) {
        Index Dofs(5*CurrentElement,5*CurrentElement+7);
        Jacobian(Dofs,All) = Jtmp;
      }
      else { // ringstructure
        Jacobian(Index(5*CurrentElement,5*CurrentElement+4),All) = Jtmp(Index(0,4),All);
        Jacobian(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      Index Dofs(5*node,5*node+2);
      Jacobian(Dofs,All) << DiagMat(3,INIT,1.0);
    }

    cp.getFrameOfReference().setJacobianOfTranslation(frameParent->getOrientation()(0,0,2,1)*trans(Jacobian(0,0,qSize-1,1)));
    cp.getFrameOfReference().setJacobianOfRotation   (frameParent->getOrientation()(0,2,2,2)*trans(Jacobian(0,2,qSize-1,2))); 
    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }   
  }
  
  void FlexibleBody1s21RCM::init() {
    FlexibleBody<Vec>::init();

    initialized = true;

    contourR->setAWC(frameParent->getOrientation());
    contourL->setAWC(frameParent->getOrientation());

    Vec contourRBinormal(3,INIT,0.0); contourRBinormal(2) = 1.0;
    Vec contourLBinormal = - contourRBinormal;

    contourR->setCb(contourRBinormal);
    contourL->setCb(contourLBinormal);

    contourR->setAlphaStart(0); contourR->setAlphaEnd(L);
    contourL->setAlphaStart(0); contourL->setAlphaEnd(L);
    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++)
        contourNodes(i) = L/Elements * i; // search area for each finite element contact search
      contourR->setNodes(contourNodes);
      contourL->setNodes(contourNodes);
    }
    else {
      contourR->setNodes(userContourNodes);
      contourL->setNodes(userContourNodes);
    }

    l0 = L/Elements;
    Vec g = trans(frameParent->getOrientation()(0,0,2,1))*ds->getAccelerationOfGravity();
    for(int i=0;i<Elements;i++) {
      qElement.push_back(Vec(8,INIT,0.));
      uElement.push_back(Vec(8,INIT,0.));
      discretization.push_back(new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g));
      if(rc != 0) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
      dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
    }

#ifdef HAVE_AMVIS
    if(getPlotFeature(amvis)==enabled) {
      ElasticBody1s21RCM *RCMbody = new ElasticBody1s21RCM(fullName,Elements,openStructure,1,boolAMVisBinary);
      RCMbody->setElementLength(l0);

      float amvisJT[3][2], amvisJR[3];
      for(int i=0;i<3;i++) {
        for(int j=0;j<2;j++) amvisJT[i][j] = frameParent->getOrientation()(i,j);
        amvisJR[i] = frameParent->getOrientation()(i,0);
      }
      RCMbody->setJacobians(amvisJT,amvisJR);
      RCMbody->setInitialTranslation(frameParent->getPosition()(0),frameParent->getPosition()(1),frameParent->getPosition()(2));
      RCMbody->setCylinder(AMVisRadius);
      RCMbody->setCuboid(AMVisBreadth,AMVisHeight);
      RCMbody->setColor(AMVisColor);

      bodyAMVis = RCMbody;
    } 
#endif
  }

  void FlexibleBody1s21RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 5*n+3; 
    else qSize = 5*n;
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody1s21RCM::setCurlRadius(double r) {
    rc = r;
    if(initialized) 
      for(int i=0;i<Elements;i++) 
        dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
  }

  void FlexibleBody1s21RCM::setMaterialDamping(double d) {
    dm = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
  }

  void FlexibleBody1s21RCM::setLehrDamping(double d) {
    dl = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) dynamic_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
  }

  void FlexibleBody1s21RCM::initRelaxed(double alpha) {
    if(!initialized) {
      if(Elements==0) throw(new MBSimError("ERROR (FlexibleBody1s21RCM::initRelaxed): Set number of finite elements!"));
      Vec q0Dummy(q0.size(),INIT,0.);
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(5*i+0,5*i+1) = direction*L/Elements*i;
          q0Dummy(5*i+2)       = alpha;
        }
      } 
      else {
        double R  = L/(2*M_PI);
        double a_ = sqrt(R*R + (L/Elements*L/Elements)/16.) - R;

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(5*i+0) = R*cos(alpha_);
          q0Dummy(5*i+1) = R*sin(alpha_);
          q0Dummy(5*i+2) = alpha_ + M_PI/2.;
          q0Dummy(5*i+3) = a_;
          q0Dummy(5*i+4) = a_;
        }
        setq0(q0Dummy);
        setu0(Vec(q0Dummy.size(),INIT,0.));
      }
    }
  }

  double FlexibleBody1s21RCM::BuildElement(const double& sGlobal) {
    double remainder = fmod(sGlobal,L);
    if(sGlobal<0.) remainder += L; // project into periodic structure 

    CurrentElement = int(remainder/l0);   
    double sLokal = remainder - (0.5 + CurrentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(CurrentElement >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam
      if(openStructure) { 
        CurrentElement =  Elements-1;
        sLokal += l0;
      }
      else
        CurrentElement -= Elements;
    }
    return sLokal;
  }

}

