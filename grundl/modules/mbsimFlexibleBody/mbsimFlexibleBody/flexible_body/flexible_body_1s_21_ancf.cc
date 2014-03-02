/* Copyright (C) 2004-2014 MBSim Development Team
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

#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_ancf.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_21_ancf.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"

#ifdef HAVE_NURBS
#include "nurbs++/nurbs.h"
#include "nurbs++/vector.h"

using namespace PLib;
#endif


using namespace fmatvec;
using namespace std;
using namespace MBSim;


namespace MBSimFlexibleBody {


  FlexibleBody1s21ANCF::FlexibleBody1s21ANCF(const string &name, bool openStructure_) : FlexibleBodyContinuum<double>(name), L(0), l0(0), E(0), A(0), I(0), rho(0), openStructure(openStructure_), initialized(false) {

    contour1sFlexible = new Contour1sFlexible("Contour1sFlexible");
    addContour(contour1sFlexible);
  }

  void FlexibleBody1s21ANCF::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 4 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloVec(j,j+7) += locVec;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloVec(j,j+3) += locVec(0,3);
      gloVec(0,  3) += locVec(4,7);
    }
  }

  void FlexibleBody1s21ANCF::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 4 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7),Index(j,j+7)) += locMat;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+3),Index(j,j+3)) += locMat(Index(0,3),Index(0,3));
      gloMat(Index(j,j+3),Index(0,3)) += locMat(Index(0,3),Index(4,7));
      gloMat(Index(0,3),Index(j,j+3)) += locMat(Index(4,7),Index(0,3));
      gloMat(Index(0,3),Index(0,3)) += locMat(Index(4,7),Index(4,7));
    }
  }

  void FlexibleBody1s21ANCF::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    int j = 4 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7)) += locMat;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(Index(j,j+3))            += locMat(Index(0,3));
      gloMat(Index(j,j+3),Index(0,3)) += locMat(Index(0,3),Index(4,7));
      gloMat(Index(0,3))              += locMat(Index(4,7));
    }
  }

  void FlexibleBody1s21ANCF::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      Vec X = computeState(cp.getLagrangeParameterPosition()(0));

      Vec tmp(3,NONINIT);
      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(R->getPosition() + R->getOrientation() * tmp);
      }
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = cos(X(2)); tmp(1) = sin(X(2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().set(1, R->getOrientation() * tmp); // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(X(2)); tmp(1) = cos(X(2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().set(0, R->getOrientation() * tmp); // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().set(2, -R->getOrientation().col(2)); // binormal (cartesian system)

      if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = X(3); tmp(1) = X(4); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(R->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = X(5);
        cp.getFrameOfReference().setAngularVelocity(R->getOrientation() * tmp);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      const int &node = cp.getNodeNumber();

      Vec tmp(3,NONINIT);

      if(ff==position || ff==position_cosy || ff==all) {
        tmp(0) = q(4*node+0); tmp(1) = q(4*node+1); tmp(2) = 0.; // temporary vector used for compensating planar description
        cp.getFrameOfReference().setPosition(R->getPosition() + R->getOrientation() * tmp);
      }

      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) =  cos(q(4*node+2)); tmp(1) = sin(q(4*node+2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().set(1, R->getOrientation() * tmp); // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = -sin(q(4*node+2)); tmp(1) = cos(q(4*node+2)); tmp(2) = 0.;
        cp.getFrameOfReference().getOrientation().set(0, R->getOrientation() * tmp); // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().set(2, -R->getOrientation().col(2)); // binormal (cartesian system)

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        tmp(0) = u(4*node+0); tmp(1) = u(4*node+1); tmp(2) = 0.;
        cp.getFrameOfReference().setVelocity(R->getOrientation() * tmp);
      }

      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        tmp(0) = 0.; tmp(1) = 0.; tmp(2) = u(4*node+2);
        cp.getFrameOfReference().setAngularVelocity(R->getOrientation() * tmp);
      }
    }
    else throw MBSimError("ERROR(FlexibleBody1sANCF::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s21ANCF::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,3-1);
    Mat Jacobian(qSize,3,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement);
      Mat Jtmp = static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->JGeneralized(qElement[currentElement],sLocal);
      if(currentElement<Elements-1 || openStructure) {
        Jacobian(Index(4*currentElement,4*currentElement+7),All) = Jtmp;
      }
      else { // ringstructure
        Jacobian(Index(4*currentElement,4*currentElement+4),All) = Jtmp(Index(0,3),All);
        Jacobian(Index(               0,                 2),All) = Jtmp(Index(4,7),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      Jacobian(Index(4*node,4*node+2),All) << DiagMat(3,INIT,1.0);
    }
    else throw MBSimError("ERROR(FlexibleBody1s21ANCF::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(R->getOrientation()(Index(0,2),Index(0,1))*Jacobian(Index(0,qSize-1),Index(0,1)).T());
    cp.getFrameOfReference().setJacobianOfRotation   (R->getOrientation()(Index(0,2),Index(2,2))*Jacobian(Index(0,qSize-1),Index(2,2)).T());

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBody1s21ANCF::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialized = true;

      contour1sFlexible->getFrame()->setOrientation(R->getOrientation());
      contour1sFlexible->setAlphaStart(0); contour1sFlexible->setAlphaEnd(L);
      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; // search area for each finite element contact search
        contour1sFlexible->setNodes(contourNodes);
      }
      else contour1sFlexible->setNodes(userContourNodes);

      l0 = L/Elements;
      Vec g = R->getOrientation()(Index(0,2),Index(0,1)).T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for(int i=0;i<Elements;i++) {
        qElement.push_back(Vec(8,INIT,0.));
        uElement.push_back(Vec(8,INIT,0.));
        discretization.push_back(new FiniteElement1s21ANCF(l0, A*rho, E*A, E*I, g));
      }

    }
    else if(stage==MBSim::plot) {
      for(int i=0;i<plotElements.size();i++) {
        plotColumns.push_back("x1 ("+numtostr(plotElements(i))+")"); // 0
        plotColumns.push_back("y1 ("+numtostr(plotElements(i))+")"); // 1
        plotColumns.push_back("dx1 ("+numtostr(plotElements(i))+")"); // 2
        plotColumns.push_back("dy1 ("+numtostr(plotElements(i))+")"); // 3
        plotColumns.push_back("x2 ("+numtostr(plotElements(i))+")"); // 4
        plotColumns.push_back("y2 ("+numtostr(plotElements(i))+")"); // 5
        plotColumns.push_back("dx2 ("+numtostr(plotElements(i))+")"); // 6
        plotColumns.push_back("dy2 ("+numtostr(plotElements(i))+")"); // 7
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      ((OpenMBV::SpineExtrusion*)openMBVBody)->setInitialRotation(AIK2Cardan(R->getOrientation()));
#endif
      FlexibleBodyContinuum<double>::init(stage);
    }
    else
      FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1s21ANCF::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);

          Vec tmp(3,NONINIT); tmp(0) = X(0); tmp(1) = X(1); tmp(2) = 0.; // temporary vector used for compensating planar description
          Vec pos = R->getPosition() + R->getOrientation() * tmp;
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

  void FlexibleBody1s21ANCF::setNumberElements(int n){
    Elements = n;
    if(openStructure) {
      qSize = 4 * (n+1);
    } else {
      qSize = 4 *  n   ;
    }
    uSize[0] = qSize;
    uSize[1] = qSize;
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  Vec FlexibleBody1s21ANCF::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->StateBalken(qElement[currentElement],uElement[currentElement],sLocal);
  }

  void FlexibleBody1s21ANCF::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int n = 4 * i ;

      if(i<Elements-1 || openStructure==true) {
        qElement[i] << q(n,n+7);
        uElement[i] << u(n,n+7);
      }
      else { // last finite element and ring closure
        qElement[i](0,3) << q(n,n+3);
        uElement[i](0,3) << u(n,n+3);
        qElement[i](4,7) << q(0,3);
        if(qElement[i](2)-q(2)>0.0) qElement[i](7) += 2*M_PI;
        else qElement[i](7) -= 2*M_PI;
        uElement[i](4,7) << u(0,3);
      }
    }
  }

  void FlexibleBody1s21ANCF::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);
    sLocal = remainder - (currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    // contact solver computes too large sGlobal at the end of the entire beam is not considered only for open structure
    // for closed structure even sGlobal < L (but sGlobal ~ L) values could lead - due to numerical problems - to a wrong currentElement computation
    if(currentElement >= Elements) {
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }

  void FlexibleBody1s21ANCF::initRelaxed(double alpha) {
    if(!initialized) {
      if(Elements==0)
        throw(new MBSimError("ERROR (FlexibleBody1s21ANCF::initRelaxed): Set number of finite elements!"));
      Vec q0Dummy(q0.size(),INIT,0.);
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(4*i+0,4*i+1) = direction*double(L/Elements*i);
          q0Dummy(4*i+2)       = alpha;
        }
      }
      else {
        double R  = L/(2*M_PI);
        double a_ = sqrt(R*R + (L/Elements*L/Elements)/16.) - R;

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(4*i+0) = R*cos(alpha_);
          q0Dummy(4*i+1) = R*sin(alpha_);
          q0Dummy(4*i+2) = alpha_ + M_PI/2.;
          q0Dummy(4*i+3) = a_;
          q0Dummy(4*i+4) = a_;
        }
      }
      setq0(q0Dummy);
      setu0(Vec(q0Dummy.size(),INIT,0.));
    }
  }

}

