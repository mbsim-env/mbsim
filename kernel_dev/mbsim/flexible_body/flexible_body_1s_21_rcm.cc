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
#include <mbsim/contours/contour1s_flexible.h>
#include <mbsim/environment.h>

#define FMATVEC_DEEP_COPY

using namespace fmatvec;
using namespace std;

namespace MBSim {

  FlexibleBody1s21RCM::FlexibleBody1s21RCM(const string &name, bool openStructure_) : FlexibleBodyContinuum<double>(name), L(0), l0(0), E(0), A(0), I(0), rho(0), rc(0), dm(0), dl(0), openStructure(openStructure_), initialized(false) { 
    contour1sFlexible = new Contour1sFlexible("Contour1sFlexible");
    Body::addContour(contour1sFlexible);
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
        if(qElement[i](2)-q(2)>0.0) qElement[i](7) += 2*M_PI;
        else qElement[i](7) -= 2*M_PI;
        uElement[i](5,7) << u(0,2);
      } 
    }
  }

  void FlexibleBody1s21RCM::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloVec(j,j+7) += locVec;
    } 
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloVec(j,j+4) += locVec(0,4);
      gloVec(0,  2) += locVec(5,7);
    }
  }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7),Index(j,j+7)) += locMat;
    } 
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+4),Index(j,j+4)) += locMat(Index(0,4),Index(0,4));
      gloMat(Index(j,j+4),Index(0,2)) += locMat(Index(0,4),Index(5,7));
      gloMat(Index(0,2),Index(j,j+4)) += locMat(Index(5,7),Index(0,4));
      gloMat(Index(0,2),Index(0,2)) += locMat(Index(5,7),Index(5,7));
    }
  }

  void FlexibleBody1s21RCM::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    int j = 5 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7)) += locMat;
    } 
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+4))            += locMat(Index(0,4));
      gloMat(Index(j,j+4),Index(0,2)) += locMat(Index(0,4),Index(5,7));
      gloMat(Index(0,2))              += locMat(Index(5,7));
    }
  }

  void FlexibleBody1s21RCM::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      Vec X = computeState(cp.getLagrangeParameterPosition()(0));

      Vec tmp(3,NONINIT);
      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = cos(X(2)); tmp(1) = sin(X(2)); tmp(2) = 0.; 
        cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation() * tmp; // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(X(2)); tmp(1) = cos(X(2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation() * tmp; // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = -frameOfReference->getOrientation().col(2); // binormal (cartesian system)

      if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = X(3); tmp(1) = X(4); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = X(5);
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * tmp);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT);

      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = q(5*node+0); tmp(1) = q(5*node+1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp);
      }

      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) =  cos(q(5*node+2)); tmp(1) = sin(q(5*node+2)); tmp(2) = 0.; 
        cp.getFrameOfReference().getOrientation().col(1)    = frameOfReference->getOrientation() * tmp; // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(q(5*node+2)); tmp(1) = cos(q(5*node+2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().col(0)    =  frameOfReference->getOrientation() * tmp; // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = -frameOfReference->getOrientation().col(2); // binormal (cartesian system)

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = u(5*node+0); tmp(1) = u(5*node+1); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = u(5*node+2);
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * tmp);
      }
    }
    else throw new MBSimError("ERROR(FlexibleBody1s21RCM::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

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
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement);
      Mat Jtmp = static_cast<FiniteElement1s21RCM*>(discretization[currentElement])->JGeneralized(qElement[currentElement],sLocal);
      if(currentElement<Elements-1 || openStructure) {
        Jacobian(Index(5*currentElement,5*currentElement+7),All) = Jtmp;
      }
      else { // ringstructure
        Jacobian(Index(5*currentElement,5*currentElement+4),All) = Jtmp(Index(0,4),All);
        Jacobian(Index(               0,                 2),All) = Jtmp(Index(5,7),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      Jacobian(Index(5*node,5*node+2),All) << DiagMat(3,INIT,1.0);
    }
    else throw new MBSimError("ERROR(FlexibleBody1s21RCM::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()(0,0,2,1)*trans(Jacobian(0,0,qSize-1,1)));
    cp.getFrameOfReference().setJacobianOfRotation   (frameOfReference->getOrientation()(0,2,2,2)*trans(Jacobian(0,2,qSize-1,2)));

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
    FlexibleBodyContinuum<double>::init();

    initialized = true;

    contour1sFlexible->getFrame()->setOrientation(frameOfReference->getOrientation());

    contour1sFlexible->setAlphaStart(0); contour1sFlexible->setAlphaEnd(L);
    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; // search area for each finite element contact search
      contour1sFlexible->setNodes(contourNodes);
    }
    else contour1sFlexible->setNodes(userContourNodes);

    l0 = L/Elements;
    Vec g = trans(frameOfReference->getOrientation()(0,0,2,1))*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
    for(int i=0;i<Elements;i++) {
      qElement.push_back(Vec(8,INIT,0.));
      uElement.push_back(Vec(8,INIT,0.));
      discretization.push_back(new FiniteElement1s21RCM(l0, A*rho, E*A, E*I, g));
      if(rc != 0) static_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
      static_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
      static_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
    }
  }

  void FlexibleBody1s21RCM::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
          Vec tmp(3,NONINIT); tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
          Vec pos = frameOfReference->getPosition() + frameOfReference->getOrientation() * tmp;
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(0.); // local twist
        }
        ((OpenMBV::SpineExtrusion*)openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
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
        static_cast<FiniteElement1s21RCM*>(discretization[i])->setCurlRadius(rc);
  }

  void FlexibleBody1s21RCM::setMaterialDamping(double d) {
    dm = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21RCM*>(discretization[i])->setMaterialDamping(dm);
  }

  void FlexibleBody1s21RCM::setLehrDamping(double d) {
    dl = d;
    if(initialized) 
      for(int i=0;i<Elements;i++) static_cast<FiniteElement1s21RCM*>(discretization[i])->setLehrDamping(dl);
  }

  Vec FlexibleBody1s21RCM::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s21RCM*>(discretization[currentElement])->StateBeam(qElement[currentElement],uElement[currentElement],sLocal);
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

  void FlexibleBody1s21RCM::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);   
    sLocal = remainder - (0.5 + currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }

}

