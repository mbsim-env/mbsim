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
#include "mbsim/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsim/flexible_body/finite_elements/finite_element_1s_23_bta.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/environment.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  FlexibleBody1s23BTA::FlexibleBody1s23BTA(const string &name) : FlexibleBodyContinuum<double>(name), L(0), l0(0.), E(0), A(0), Iyy(0), Izz(0), rho(0), rc(0) { 
    cylinderFlexible = new CylinderFlexible("CylinderFlexible");
    Body::addContour(cylinderFlexible);
  }

  void FlexibleBody1s23BTA::BuildElements() {
    for(int i=0;i<Elements;i++) {
      Index activeElement( discretization[i]->getuSize()/2*i , discretization[i]->getuSize()/2*(i+2) -1 );
      qElement[i] = q(activeElement);
      uElement[i] = u(activeElement);
    }
  }

  void FlexibleBody1s23BTA::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloVec(activeElement) += locVec;
  }

  void FlexibleBody1s23BTA::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloMat(activeElement) += locMat;
  }

  void FlexibleBody1s23BTA::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    Index activeElement( discretization[n]->getuSize()/2*n , discretization[n]->getuSize()/2*(n+2) -1 );
    gloMat(activeElement) += locMat;
  }

  void FlexibleBody1s23BTA::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == NODE) { // frame on node
      cp.getContourParameterType() = CONTINUUM;
      cp.getLagrangeParameterPosition() = Vec(1,INIT,cp.getNodeNumber()*L/Elements);
    }

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0),sLocal,currentElement);
      Vec X;
      if(ff==position || ff==position_cosy || ff==velocity || ff==angularVelocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
        X = static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->StateAxis(qElement[currentElement],uElement[currentElement],sLocal); 
        X(0) = cp.getLagrangeParameterPosition()(0);
      }
      SqrMat AWK;
      if(ff==firstTangent || ff==normal || ff==cosy || ff==secondTangent || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        AWK = frameOfReference->getOrientation()*static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->AWK(qElement[currentElement],sLocal);
      }

      if(ff==position || ff==position_cosy || ff==all) {
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * X(0,2));
      }
      if(ff==firstTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        cp.getFrameOfReference().getOrientation().col(1) = AWK.col(0); // tangent
      }
      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        cp.getFrameOfReference().getOrientation().col(0) = AWK.col(1); // normal
      }
      if(ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = -AWK.col(2); // binormal (cartesian system)
      if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * X(6,8));
      }
      if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * X(9,11));
      }
    }
    else throw new MBSimError("ERROR(FlexibleBody1s23BTA::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition       (cp.getFrameOfReference().getPosition());
      frame->setOrientation    (cp.getFrameOfReference().getOrientation());
      frame->setVelocity       (cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s23BTA::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,5-1);
    Mat Jacobian(qSize,5);

    if(cp.getContourParameterType() == NODE) { // force on node
      cp.getContourParameterType() = CONTINUUM;
      cp.getLagrangeParameterPosition() = Vec(1,INIT,cp.getNodeNumber()*L/Elements);
    }

    if(cp.getContourParameterType() == CONTINUUM) { // force on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement);

      Index activeElement( discretization[currentElement]->getuSize()/2*currentElement, discretization[currentElement]->getuSize()/2*(currentElement+2) -1 );
      Jacobian(activeElement,All) = static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->JGeneralized(qElement[currentElement],sLocal);
    }
    else throw new MBSimError("ERROR(FlexibleBody1s23BTA::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()(0,1,2,2)*Jacobian(0,0,qSize-1,1).T());
    cp.getFrameOfReference().setJacobianOfRotation   (frameOfReference->getOrientation()*Jacobian(0,2,qSize-1,4).T());

    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation   (cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation   (cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }   
  }

  void FlexibleBody1s23BTA::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      assert(0<Elements);

      cylinderFlexible->getFrame()->setOrientation(frameOfReference->getOrientation());
      cylinderFlexible->setAlphaStart(0);  cylinderFlexible->setAlphaEnd(L);
      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; 
        cylinderFlexible->setNodes(contourNodes);
      }
      else cylinderFlexible->setNodes(userContourNodes);

      l0 = L/Elements;
      Vec g = frameOfReference->getOrientation().T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      for(int i=0; i<Elements; i++) {
        discretization.push_back(new FiniteElement1s23BTA(l0, A*rho, E*Iyy, E*Izz, It*rho, G*It, g ));
        qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
        uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
        static_cast<FiniteElement1s23BTA*>(discretization[i])->setTorsionalDamping(dTorsional);
      }
    }
    else
      FlexibleBodyContinuum<double>::init(stage);    
  }

  void FlexibleBody1s23BTA::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          double sLocal;
          int currentElement;
          BuildElement(ds*i,sLocal,currentElement);
          Vec X = static_cast<FiniteElement1s23BTA*>(discretization[currentElement])->StateAxis(qElement[currentElement],uElement[currentElement],sLocal); 
          X(0) = ds*i;
          Vec pos = frameOfReference->getPosition() + frameOfReference->getOrientation() * X(0,2);
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }
        ((OpenMBV::SpineExtrusion*)openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
  }  

  void FlexibleBody1s23BTA::setNumberElements(int n) {
    Elements = n;
    qSize = 5*( Elements + 1 ); 
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody1s23BTA::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    currentElement = int(sGlobal/l0);   
    sLocal = sGlobal - currentElement * l0; // Lagrange-Parameter of the affected FE 

    if(currentElement >= Elements) { // contact solver computes to large sGlobal at the end of the entire beam
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }

}

