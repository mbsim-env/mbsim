/* Copyright (C) 2004-2011 MBSim Development Team
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

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"
#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1s33Cosserat::FlexibleBody1s33Cosserat(const string &name, bool openStructure_) : FlexibleBodyContinuum<double> (name), cylinder(new CylinderFlexible("Cylinder")), top(new FlexibleBand("Top")), bottom(new FlexibleBand("Bottom")), left(new FlexibleBand("Left")), right(new FlexibleBand("Right")), angle(new Cardan()), Elements(0), L(0.), l0(0.), E(0.), G(0.),A(0.), I1(0.), I2(0.), I0(0.), rho(0.), R1(0.), R2(0.), cEps0D(0.), cEps1D(0.), cEps2D(0.), openStructure(openStructure_), initialised(false), bound_ang_start(3,INIT,0.), bound_ang_end(3,INIT,0.), bound_ang_vel_start(3,INIT,0.), bound_ang_vel_end(3,INIT,0.), cuboidBreadth(0.), cuboidHeight(0.), cylinderRadius(0.), curve(new NurbsCurve1s("Curve")) {
    Body::addContour(cylinder);
    Body::addContour(top);
    Body::addContour(bottom);
    Body::addContour(left);
    Body::addContour(right);
    Body::addContour(curve);
  }

  FlexibleBody1s33Cosserat::~FlexibleBody1s33Cosserat() {
    for(unsigned int i=0; i<rotationDiscretization.size(); i++) {
      if(rotationDiscretization[i]) { delete rotationDiscretization[i]; rotationDiscretization[i] = NULL; }
    }
  }

  void FlexibleBody1s33Cosserat::BuildElements() {
    /* translational elements */
    for(int i=0;i<Elements;i++) {
      int j = 6*i; // start index in entire beam coordinates

      if(i<Elements-1 || openStructure) {
        qElement[i] = q(j,j+8);
        uElement[i] = u(j,j+8);
      }
      else { // last FE-Beam for closed structure	
        qElement[i](0,5) = q(j,j+5);
        uElement[i](0,5) = u(j,j+5);
        qElement[i](6,8) = q(0,2);
        uElement[i](6,8) = u(0,2);
      }
    }

    /* rotational elements */
    if(openStructure) computeBoundaryCondition();

    for(int i=0;i<Elements+1;i++) {
      int j = 6*i; // start index in entire beam coordinates

      if(i>0 && i<Elements) { // no problem case
        qRotationElement[i] = q(j-3,j+5); // staggered grid -> rotation offset 
        uRotationElement[i] = u(j-3,j+5);
      }
      else if(i==0) { // first element 
        if(openStructure) { // open structure
          qRotationElement[i](0,2) = bound_ang_start;
          uRotationElement[i](0,2) = bound_ang_vel_start;
          qRotationElement[i](3,8) = q(j,j+5);
          uRotationElement[i](3,8) = u(j,j+5);
        }
        else { // closed structure concerning gamma
          qRotationElement[i](0,2) = q(q.size()-3,q.size()-1); 
          uRotationElement[i](0,2) = u(u.size()-3,u.size()-1); 
          qRotationElement[i](3,8) = q(j,j+5);
          uRotationElement[i](3,8) = u(j,j+5);
          if(q(j+5)<q(q.size()-1)) qRotationElement[i](2) -= 2.*M_PI;
          else qRotationElement[i](2) += 2.*M_PI;
        }
      }
      else if(i==Elements) { // last element
        if(openStructure) { // open structure
          qRotationElement[i](0,5) = q(j-3,j+2);
          uRotationElement[i](0,5) = u(j-3,j+2);
          qRotationElement[i](6,8) = bound_ang_end;
          uRotationElement[i](6,8) = bound_ang_vel_end;
        }
        else { // closed structure concerning gamma
          qRotationElement[i](0,2) = q(j-3,j-1); 
          uRotationElement[i](0,2) = u(j-3,j-1);
          qRotationElement[i](3,8) = q(0,5);
          uRotationElement[i](3,8) = u(0,5);
          if(q(j-1)<q(5)) qRotationElement[i](8) -= 2.*M_PI;
          else qRotationElement[i](8) += 2.*M_PI;
        }
      }
    }
  }

  void FlexibleBody1s33Cosserat::GlobalVectorContribution(int n, const Vec& locVec,Vec& gloVec) {
    int j = 6*n; // start index in entire beam coordinates
    
    if(n<Elements-1 || openStructure) {
      gloVec(j,j+8) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j,j+5) += locVec(0,5);
      gloVec(0,2) += locVec(6,8);
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const Mat& locMat, Mat& gloMat) {
    int j = 6*n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+8),Index(j,j+8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+5),Index(j,j+5)) += locMat(Index(0,5),Index(0,5)); 
      gloMat(Index(j,j+5),Index(0,2)) += locMat(Index(0,5),Index(6,8));
      gloMat(Index(0,2),Index(j,j+5)) += locMat(Index(6,8),Index(0,5));
      gloMat(Index(0,2),Index(0,2)) += locMat(Index(6,8),Index(6,8));
    }
  }

  void FlexibleBody1s33Cosserat::GlobalMatrixContribution(int n, const SymMat& locMat, SymMat& gloMat) {
    int j = 6*n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      gloMat(Index(j,j+8)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(Index(j,j+5)) += locMat(Index(0,5)); 
      gloMat(Index(j,j+5),Index(0,2)) += locMat(Index(0,5),Index(6,8));
      gloMat(Index(0,2)) += locMat(Index(6,8));
    }
  }
  
  void FlexibleBody1s33Cosserat::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
#ifdef HAVE_NURBS
      curve->updateKinematicsForFrame(cp,ff);
#endif
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();

      if(ff==position) cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation()*q(6*node+0,6*node+2));
      if(ff==velocity) cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*u(6*node+0,6*node+2));
    }
    else throw MBSimError("ERROR(FlexibleBody1s33Cosserat::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s33Cosserat::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,5);
    Index One(0,2);
    Mat Jacobian(qSize,6,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement); // compute parameters of affected FE
      Mat Jtmp = static_cast<FiniteElement1s33CosseratTranslation*>(discretization[currentElement])->computeJacobianOfMotion(qElement[currentElement],sLocal); // this local ansatz yields continuous and finite wave propagation 

      if(currentElement<Elements-1 || openStructure) {
        Jacobian(Index(10*currentElement,10*currentElement+15),All) = Jtmp;
      }
      else { // last FE for closed structure
        Jacobian(Index(10*currentElement,10*currentElement+9),All) = Jtmp(Index(0,9),All);
        Jacobian(Index(0,5),All) = Jtmp(Index(10,15),All);
      }
    }
    else throw MBSimError("ERROR(FlexibleBody1s33RCM::updateJacobiansForFrame): ContourPointDataType should be 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()*Jacobian(0,0,qSize-1,2).T());
    cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation()*Jacobian(0,3,qSize-1,5).T()); 
    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  void FlexibleBody1s33Cosserat::init(InitStage stage) {
    if(stage == unknownStage) {
      FlexibleBodyContinuum<double>::init(stage);

      initialised = true;

      /* cylinder */
      cylinder->setAlphaStart(0.);
      cylinder->setAlphaEnd(L);

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements*i; // own search area for each element
        cylinder->setNodes(contourNodes);
      }
      else {
        cylinder->setNodes(userContourNodes);
      }

      cylinder->setRadius(cylinderRadius);

      /* cuboid */
      top->setCn(Vec("[1.;0.]"));
      bottom->setCn(Vec("[-1.;0.]"));
      left->setCn(Vec("[0.;-1.]"));
      right->setCn(Vec("[0.;1.]"));

      top->setAlphaStart(0.);
      top->setAlphaEnd(L);

      bottom->setAlphaStart(0.);
      bottom->setAlphaEnd(L);

      left->setAlphaStart(0.);
      left->setAlphaEnd(L);

      right->setAlphaStart(0.);
      right->setAlphaEnd(L);

      if(userContourNodes.size()==0) {
        Vec contourNodes(Elements+1);
        for(int i=0;i<=Elements;i++) contourNodes(i) = L / Elements * i;
        top->setNodes(contourNodes);
        bottom->setNodes(contourNodes);
        left->setNodes(contourNodes);
        right->setNodes(contourNodes);
      }
      else {
        top->setNodes(userContourNodes);
        bottom->setNodes(userContourNodes);
        left->setNodes(userContourNodes);
        right->setNodes(userContourNodes);
      }

      top->setWidth(cuboidBreadth);
      bottom->setWidth(cuboidBreadth);
      top->setNormalDistance(0.5*cuboidHeight);
      bottom->setNormalDistance(0.5*cuboidHeight);
      left->setWidth(cuboidHeight);
      right->setWidth(cuboidHeight);
      left->setNormalDistance(0.5*cuboidBreadth);
      right->setNormalDistance(0.5*cuboidBreadth);

      l0 = L / Elements;
      Vec g = frameOfReference->getOrientation().T()* MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      
      /* translational elements */
      for(int i=0;i<Elements;i++) {
        discretization.push_back(new FiniteElement1s33CosseratTranslation(l0,rho,A,E,G,I1,I2,I0,g,angle));
        qElement.push_back(Vec(discretization[i]->getqSize(),INIT,0.));
        uElement.push_back(Vec(discretization[i]->getuSize(),INIT,0.));
        static_cast<FiniteElement1s33CosseratTranslation*>(discretization[i])->setMaterialDamping(Elements*cEps0D,cEps1D,cEps2D);
      }

      /* rotational elements */
      for(int i=0;i<Elements+1;i++) { 
        rotationDiscretization.push_back(new FiniteElement1s33CosseratRotation(l0,E,G,I1,I2,I0,angle));
        qRotationElement.push_back(Vec(rotationDiscretization[i]->getqSize(),INIT,0.));
        uRotationElement.push_back(Vec(rotationDiscretization[i]->getuSize(),INIT,0.));
        if(fabs(R1)>epsroot() || fabs(R2)>epsroot()) static_cast<FiniteElement1s33CosseratRotation*>(rotationDiscretization[i])->setCurlRadius(R1,R2);
      }

      initM();
    }

    else FlexibleBodyContinuum<double>::init(stage);

#ifdef HAVE_NURBS
    curve->initContourFromBody(stage);
#endif
  }

  double FlexibleBody1s33Cosserat::computePotentialEnergy() {
    /* translational elements */
    double V = FlexibleBodyContinuum<double>::computePotentialEnergy();

    /* rotational elements */
    for(unsigned int i=0;i<rotationDiscretization.size();i++) {
      V += rotationDiscretization[i]->computeElasticEnergy(qRotationElement[i]);
    }

    return V;
  }

  void FlexibleBody1s33Cosserat::facLLM(int k) {
    for(int i=0;i<(int)discretization.size();i++) {
      int j = 6*i; 
      LLM[k](Index(j+3,j+5)) = facLL(discretization[i]->getM()(Index(3,5)));
    }
  }

  void FlexibleBody1s33Cosserat::updateh(double t, int k) {
    /* translational elements */
    FlexibleBodyContinuum<double>::updateh(t);

    /* rotational elements */
    for(int i=0;i<(int)rotationDiscretization.size();i++) {
      try { rotationDiscretization[i]->computeh(qRotationElement[i],uRotationElement[i]); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)rotationDiscretization.size();i++) GlobalVectorContributionRotation(i,rotationDiscretization[i]->geth(),h[0]); // assemble
  }

  void FlexibleBody1s33Cosserat::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<double>::updateStateDependentVariables(t);

#ifdef HAVE_NURBS
    curve->computeCurveTranslations();
    curve->computeCurveVelocities();
#endif
  }

  void FlexibleBody1s33Cosserat::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV) == enabled && openMBVBody) {

        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 1) : L/(((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints()- 2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
          Vec pos = frameOfReference->getPosition()+ frameOfReference->getOrientation() * X(0,2);
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }

        ((OpenMBV::SpineExtrusion*) openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<double>::plot(t,dt);
  }

  void FlexibleBody1s33Cosserat::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 6*n+3;
    else qSize = 6*n;

    Vec q0Tmp(0,INIT,0.);
    if(q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize,INIT,0.);
    if(q0Tmp.size()) {
      if(q0Tmp.size()==q0.size())
        q0 = q0Tmp.copy();
      else
        throw MBSimError("Error in dimension of q0 of FlexibleBody1s33Cosserat \"" + name + "\"!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    Vec u0Tmp(0,INIT,0);
    if(u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0],INIT,0.);
    if(u0Tmp.size()) {
      if(u0Tmp.size() == u0.size())
        u0 = u0Tmp.copy();
      else
        throw MBSimError("Error in dimension of u0 of FlexibleBody1s33Cosserat \"" + name  + "\"!");
    }
  }

  Vec FlexibleBody1s33Cosserat::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal,sLocal,currentElement); // Lagrange parameter of affected FE
    Vec temp = static_cast<FiniteElement1s33CosseratTranslation*> (discretization[currentElement])->computeState(qElement[currentElement],uElement[currentElement],sLocal);

    //ContourPointData cp(sGlobal);
    //updateKinematicsForFrame(cp,position);
    //temp(0,2) = cp.getFrameOfReference().getPosition().copy();
    //updateKinematicsForFrame(cp,velocity);
    //temp(6,8) = cp.getFrameOfReference().getVelocity().copy();
    return temp.copy();
  }

  void FlexibleBody1s33Cosserat::BuildElement(const double& sGlobal, double& sLocal,int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);
    sLocal = remainder - (currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 and sGlobal==0 at the beginning of the beam
    
    assert(sLocal>-1e-8);
    assert(sLocal<l0+1e-8);

    if(currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement = Elements - 1;
      sLocal += l0;
    }
  }
  
  void FlexibleBody1s33Cosserat::initM() {
    for(int i=0;i<(int)discretization.size();i++) {
      try { static_cast<FiniteElement1s33CosseratTranslation*>(discretization[i])->initM(); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)discretization.size();i++) GlobalMatrixContribution(i,discretization[i]->getM(),M[0]); // assemble
    for(int i=0;i<(int)discretization.size();i++) {
      int j = 6*i; 
      LLM[0](Index(j,j+2)) = facLL(M[0](Index(j,j+2)));
      if(openStructure && i==(int)discretization.size()-1)
        LLM[0](Index(j+6,j+8)) = facLL(M[0](Index(j+6,j+8)));
    }
  }
  
  void FlexibleBody1s33Cosserat::computeBoundaryCondition() {
    // TODO
  }
  
  void FlexibleBody1s33Cosserat::GlobalVectorContributionRotation(int n, const Vec& locVec,Vec& gloVec) {
    int j = 6*n; // start index in entire beam coordinates
    if(n>0 && n<Elements) { // no problem case
      gloVec(j-3,j+5) += locVec; // staggered grid -> rotation offset
    }
    else if(n==0) { // first element 
      if(openStructure) { // open structure
        gloVec(j,j+5) += locVec(3,8);
        gloVec(j+3,j+5) += locVec(0,2); // TODO depends on computeBoundaryConditions()
      }
      else { // closed structure 
        gloVec(j,j+5) += locVec(3,8);
        gloVec(q.size()-3,q.size()-1) += locVec(0,2);
      }
    }
    else if(n==Elements) { // last element
      if(openStructure) { // open structure
        gloVec(j-3,j+2) += locVec(0,5);
        gloVec(j-3,j-1) += locVec(6,8); // TODO depends on computeBoundaryConditions()
      }
      else { // closed structure
        gloVec(j-3,j-1) += locVec(0,2); 
        gloVec(0,5) += locVec(3,8);
      }
    }
  }

}

