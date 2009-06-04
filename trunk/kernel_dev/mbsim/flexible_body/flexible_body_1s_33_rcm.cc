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
 */

#include<config.h>
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsim/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/contours/flexible_band.h"
#include "mbsim/contours/cylinder_flexible.h"
#include "mbsim/flexible_body/finite_elements/finite_element_1s_33_rcm/revcardan.h"
#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  FlexibleBody1s33RCM::FlexibleBody1s33RCM(const string &name,bool openStructure_) : FlexibleBodyContinuum<double>(name),cylinder(0),top(0),bottom(0),left(0),right(0),angle(0),Elements(0),L(0.),l0(0.),E(0.),G(0.),A(0.),I1(0.),I2(0.),I0(0.),rho(0.),R1(0.),R2(0.),epstD(0.),k0D(0.),epstL(0.),k0L(0.),openStructure(openStructure_),implicit(false),CurrentElement(0),initialised(false),nGauss(3),cylinderRadius(0.),cuboidBreadth(0.),cuboidHeight(0.) {
    cylinder = new CylinderFlexible("Cylinder");
    Body::addContour(cylinder);

    top = new FlexibleBand("Top");
    Body::addContour(top);

    bottom = new FlexibleBand("Bottom");
    Body::addContour(bottom);

    left = new FlexibleBand("Left");
    Body::addContour(left);

    right = new FlexibleBand("Right");
    Body::addContour(right);

    angle = new RevCardan();
  }

  FlexibleBody1s33RCM::~FlexibleBody1s33RCM() {
    delete angle;
  }

  void FlexibleBody1s33RCM::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int j = 10 * i; // start index in entire beam coordinates

      if(i<Elements-1 || openStructure) {
        qElement[i] = q(j,j+15);
        uElement[i] = u(j,j+15);
      }
      else { // last FE-Beam for closed structure	
        qElement[i](0,9) = q(j,j+9);
        uElement[i](0,9) = u(j,j+9);
        qElement[i](10,15) = q(0,5);
        if(q(j+5)<q(5)) qElement[i](15) -= 2.*M_PI;
        else qElement[i](15) += 2.*M_PI;
        uElement[i](10,15) = u(0,5);
      }
    }
  }

  void FlexibleBody1s33RCM::GlobalMatrixContribution(int n) {
    int j = 10 * n; // start index in entire beam coordinates

    if(n<Elements-1 || openStructure) {
      M(Index(j,j+15)) += discretization[n]->getMassMatrix();
      h(j,j+15) += discretization[n]->getGeneralizedForceVector();

      //      	if(implicit) { // implicit integration
      //				Jhq(j,j,j+15,j+15) += discretization[n]->getJGq();
      //				Jhqt(j,j,j+15,j+15) += discretization[n]->getJhqt();
      //      	}
    }
    else { // last FE for closed structure
      M(Index(j,j+9)) += (discretization[n]->getMassMatrix())(Index(0,9)); // symmetric mass matrix (object.h)
      M(Index(j,j+9),Index(0,5)) += (discretization[n]->getMassMatrix())(Index(0,9),Index(10,15));
      M(Index(0,5)) += (discretization[n]->getMassMatrix())(Index(10,15));
      h(j,j+9) += (discretization[n]->getGeneralizedForceVector())(0,9);
      h(0,5) += (discretization[n]->getGeneralizedForceVector())(10,15);

      //      	if(implicit) { // implicit integration
      //				Jhq(j,j,j+9,j+9) += discretization[n]->getJhq(0,0,9,9);
      //				Jhq(j,0,j+9,5) += discretization[n]->getJhq(0,10,9,15);
      //				Jhq(0,j,5,j+9) += discretization[n]->getJhq(10,0,15,9);
      //				Jhq(0,0,5,5) += discretization[n]->getJhq(10,10,15,15);

      //				Jhqt(j,j,j+9,j+9) += discretization[n]->getJhqt(0,0,9,9);
      //				Jhqt(j,0,j+9,5) += discretization[n]->getJhqt(0,10,9,15);
      //				Jhqt(0,j,5,j+9) += discretization[n]->getJhqt(10,0,15,9);
      //				Jhqt(0,0,5,5) += discretization[n]->getJhqt(10,10,15,15);
      //      	}
    }
  }

  void FlexibleBody1s33RCM::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff, Frame *frame) {
    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      Vec X = computeState(cp.getLagrangeParameterPosition()(0)); // state of affected FE
      Vec Phi = X(3,5);
      Vec Phit = X(9,11);

      if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * X(0,2));
      if(ff==firstTangent || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation()*angle->computet(Phi); // tangent
      if(ff==normal || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation()*angle->computen(Phi); // normal
      if(ff==secondTangent || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = crossProduct(cp.getFrameOfReference().getOrientation().col(0),cp.getFrameOfReference().getOrientation().col(1)); // binormal
      if(ff==velocity || velocity_cosy || ff==velocities || velocities_cosy || ff==all) cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*X(6,8));
      if(ff==angularVelocity || ff==velocities || velocities_cosy || ff==all) cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation()*angle->computeOmega(Phi,Phit));
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();
      Vec Phi = q(10*node+3,10*node+5);
      Vec Phit = u(10*node+3,10*node+5);

      if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation()*q(10*node+0,10*node+2));
      if(ff==firstTangent || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(1) = frameOfReference->getOrientation()*angle->computet(Phi); // tangent
      if(ff==normal || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(0) = frameOfReference->getOrientation()*angle->computen(Phi); // normal
      if(ff==secondTangent || ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().getOrientation().col(2) = crossProduct(cp.getFrameOfReference().getOrientation().col(0),cp.getFrameOfReference().getOrientation().col(1)); // binormal (cartesian system)
      if(ff==velocity || ff==velocities || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation()*u(10*node+0,10*node+2));
      if(ff==angularVelocity || ff==velocities || velocity_cosy || velocities_cosy || ff==all) cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation()*angle->computeOmega(Phi,Phit));
    }
    else throw new MBSimError("ERROR(FlexibleBody1s33RCM::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if(frame!=0) { // frame should be linked to contour point data
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  void FlexibleBody1s33RCM::updateJacobiansForFrame(ContourPointData &cp, Frame *frame) {
    Index All(0,5);
    Index One(0,2);
    Mat Jacobian(qSize,6,INIT,0.);

    if(cp.getContourParameterType() == CONTINUUM) { // frame on continuum
      double sLokal = BuildElement(cp.getLagrangeParameterPosition()(0)); // compute parameters of affected FE
      Mat Jtmp = static_cast<FiniteElement1s33RCM*>(discretization[CurrentElement])->computeJXqG(qElement[CurrentElement],sLokal); // this local ansatz yields continuous and finite wave propagation 

      if(CurrentElement<Elements-1 || openStructure) {
        Jacobian(Index(10*CurrentElement,10*CurrentElement+15),All) = Jtmp;
      }
      else { // last FE for closed structure
        Jacobian(Index(10*CurrentElement,10*CurrentElement+9),All) = Jtmp(Index(0,9),All);
        Jacobian(Index(0,5),All) = Jtmp(Index(10,15),All);
      }
    }
    else if(cp.getContourParameterType() == NODE) { // frame on node
      int node = cp.getNodeNumber();

      Vec p = q(10*node+3,10*node+5);
      Vec t = angle->computet(p);
      Vec n = angle->computen(p);
      Vec b = angle->computeb(p);		
      SqrMat tp = angle->computetq(p);
      SqrMat np = angle->computenq(p);
      SqrMat bp = angle->computebq(p); 

      Jacobian(Index(10*node,10*node+2),One) << SqrMat(3,EYE); // translation
      Jacobian(Index(10*node+3,10*node+5),3) = t(1)*trans(tp(2,0,2,2))+n(1)*trans(np(2,0,2,2))+b(1)*trans(bp(2,0,2,2)); // rotation
      Jacobian(Index(10*node+3,10*node+5),4) = t(2)*trans(tp(0,0,0,2))+n(2)*trans(np(0,0,0,2))+b(2)*trans(bp(0,0,0,2));
      Jacobian(Index(10*node+3,10*node+5),5) = t(0)*trans(tp(1,0,1,2))+n(0)*trans(np(1,0,1,2))+b(0)*trans(bp(1,0,1,2)); 
    }
    else throw new MBSimError("ERROR(FlexibleBody1s33RCM::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation()*trans(Jacobian(0,0,qSize-1,2)));
    cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation()*trans(Jacobian(0,3,qSize-1,5))); 
    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if(frame!=0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }   
  }

  void FlexibleBody1s33RCM::init() {
    FlexibleBodyContinuum<double>::init();

    initialised = true;

    /* cylinder */
    cylinder->setAlphaStart(0.);
    cylinder->setAlphaEnd(L);

    if(userContourNodes.size()==0) {
      Vec contourNodes(Elements+1);
      for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i; // own search area for each element
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
      for(int i=0;i<=Elements;i++) contourNodes(i) = L/Elements * i;
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

    l0 = L/Elements;
    Vec g = trans(frameOfReference->getOrientation())*ds->getAccelerationOfGravity();

    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s33RCM(l0,rho,A,E,G,I1,I2,I0,g,angle));
      qElement.push_back(Vec(discretization[0]->getSizeOfPositions(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getSizeOfVelocities(),INIT,0.));
      static_cast<FiniteElement1s33RCM*>(discretization[i])->setGauss(nGauss);  		
      if(R1 != 0. || R2 != 0.) static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1,R2);
      static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements*epstD,Elements*k0D);
      if(epstD == 0.) static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements*epstL,Elements*k0L);
    }
  }

  void FlexibleBody1s33RCM::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-1) : L/(((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints()-2);
        for(int i=0; i<((OpenMBV::SpineExtrusion*)openMBVBody)->getNumberOfSpinePoints(); i++) {
          Vec X = computeState(ds*i);
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

  void FlexibleBody1s33RCM::setNumberElements(int n) {
    Elements = n;
    if(openStructure) qSize = 10*n+6;
    else qSize = 10*n;

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  Vec FlexibleBody1s33RCM::computeState(double sGlobal) {
    double sLocal = BuildElement(sGlobal); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[CurrentElement])->computeState(qElement[CurrentElement],uElement[CurrentElement],sLocal);
  }

  void FlexibleBody1s33RCM::initInfo() {
    l0 = L/Elements;
    Vec g = Vec("[0.;0.;0.]");
    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s33RCM(l0,rho,A,E,G,I1,I2,I0,g,angle));
      qElement.push_back(Vec(discretization[0]->getSizeOfPositions(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getSizeOfVelocities(),INIT,0.));
    }
    BuildElements();
  }

  double FlexibleBody1s33RCM::BuildElement(double sGlobal) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    CurrentElement = int(remainder/l0);   
    double sLokal = remainder - (0.5 + CurrentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if(CurrentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      CurrentElement =  Elements-1;
      sLokal += l0;
    }
    return sLokal;
  }

}

