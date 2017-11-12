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
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_ancf.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_ancf.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {


  FlexibleBody1s33ANCF::FlexibleBody1s33ANCF(const string &name, bool openStructure) : FlexibleBody1s(name,openStructure), Elements(0), l0(0.), E(0.), G(0.), A(0.), I0(0.), I1(0.), I2(0.), rho(0), rc1(0.), rc2(0.), initialised(false) { }

  void FlexibleBody1s33ANCF::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 6 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloVec(j,j+11) += locVec;
    }
    else { // ring closure at finite element (end,1)
      gloVec(j,j+5) += locVec(0,5);
      gloVec(0,  5) += locVec(6,11);
    }
  }

  void FlexibleBody1s33ANCF::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 6 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(RangeV(j,j+11),RangeV(j,j+11)) += locMat;
    }
    else { // ring closure at finite element (end,1)
      gloMat(RangeV(j,j+5),RangeV(j,j+5)) += locMat(RangeV(0,5),RangeV(0,5));
      gloMat(RangeV(j,j+5),RangeV(0,5)) += locMat(RangeV(0,5),RangeV(6,11));
      gloMat(RangeV(0,5),RangeV(j,j+5)) += locMat(RangeV(6,11),RangeV(0,5));
      gloMat(RangeV(0,5),RangeV(0,5)) += locMat(RangeV(6,11),RangeV(6,11));
    }
  }

  void FlexibleBody1s33ANCF::GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat) {
    int j = 6 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(RangeV(j,j+11)) += locMat;
    }
    else { // ring closure at finite element (end,1) with angle difference 2*M_PI
      gloMat(RangeV(j,j+5))            += locMat(RangeV(0,5));
      gloMat(RangeV(j,j+5),RangeV(0,5)) += locMat(RangeV(0,5),RangeV(6,11));
      gloMat(RangeV(0,5))              += locMat(RangeV(6,11));
    }
  }

  void FlexibleBody1s33ANCF::updatePositions(Frame1s *frame) {
    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement); // Lagrange parameter of affected FE
    frame->setPosition(R->evalPosition() + R->evalOrientation() *  static_cast<FiniteElement1s33ANCF*>(discretization[currentElement])->getPosition(getqElement(currentElement),sLocal));
    frame->setOrientation(R->evalOrientation() *  static_cast<FiniteElement1s33ANCF*>(discretization[currentElement])->getOrientation(getqElement(currentElement),sLocal));
  }

  void FlexibleBody1s33ANCF::updateVelocities(Frame1s *frame) {
    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement); // Lagrange parameter of affected FE
    frame->setVelocity(R->evalOrientation() *  static_cast<FiniteElement1s33ANCF*>(discretization[currentElement])->getVelocity(getqElement(currentElement),getuElement(currentElement),sLocal));
    frame->setAngularVelocity(R->evalOrientation() *  static_cast<FiniteElement1s33ANCF*>(discretization[currentElement])->getAngularVelocity(getqElement(currentElement),getuElement(currentElement),sLocal));
  }

  void FlexibleBody1s33ANCF::updateAccelerations(Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33ANCF::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33ANCF::updateJacobians(Frame1s *frame, int j) {
    RangeV All(0,6-1);
    Mat Jacobian(qSize,6,INIT,0.);
    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(),sLocal,currentElement);
    Mat Jtmp = static_cast<FiniteElement1s33ANCF*>(discretization[currentElement])->JGeneralized(getqElement(currentElement),sLocal);
    if(currentElement<Elements-1 || openStructure) {
      Jacobian(RangeV(6*currentElement,6*currentElement+11),All) = Jtmp;
    }
    else { // ringstructure
      Jacobian(RangeV(6*currentElement,6*currentElement+5),All) = Jtmp(RangeV(0,5),All);
      Jacobian(RangeV(               0,                 5),All) = Jtmp(RangeV(6,11),All);
    }
    frame->setJacobianOfTranslation(R->evalOrientation()*Jacobian(RangeV(0,qSize-1),RangeV(0,2)).T(),j);
    frame->setJacobianOfRotation(R->evalOrientation()*Jacobian(RangeV(0,qSize-1),RangeV(3,5)).T(),j);
  }

  void FlexibleBody1s33ANCF::updateGyroscopicAccelerations(Frame1s *frame) {
//    THROW_MBSIMERROR("(FlexibleBody1s33ANCF::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s33ANCF::updatePositions(NodeFrame *frame) {
    Vec3 tmp(NONINIT), s(NONINIT), n(NONINIT);
    int node = frame->getNodeNumber();
    tmp(0) = q(6*node+0);
    tmp(1) = q(6*node+1);
    tmp(2) = q(6*node+2);
    frame->setPosition(R->evalPosition() + R->evalOrientation() * tmp);
    s(0) = q(6*node+3);
    s(1) = q(6*node+4);
    s(2) = q(6*node+5);
    s /= nrm2(s);
    frame->getOrientation(false).set(0, R->getOrientation() * s);
    n(0) = q(6*node+3);
    n(1) = -q(6*node+4);
    n(2) = 0.;
    n /= nrm2(n);
    std::swap(n(0),n(1));
    frame->getOrientation(false).set(1, R->getOrientation() * n);
    frame->getOrientation(false).set(2, R->getOrientation() * crossProduct(s,n));
  }

  void FlexibleBody1s33ANCF::updateVelocities(NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    tmp(0) = u(6*node+0);
    tmp(1) = u(6*node+1);
    tmp(2) = u(6*node+2);
    frame->setVelocity(R->evalOrientation() * tmp);
    tmp(0) = 0.;
    tmp(1) = 0.;
    tmp(2) = 0.;
//    THROW_MBSIMERROR("(FlexibleBody1s33ANCF::updateKinematicsForFrame): angularVelocity not implemented.");
    frame->setAngularVelocity(R->evalOrientation() * tmp);
  }

  void FlexibleBody1s33ANCF::updateAccelerations(NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33ANCF::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s33ANCF::updateJacobians(NodeFrame *frame, int j) {
    RangeV All(0,6-1);
    Mat Jacobian(qSize,6,INIT,0.);
    int node = frame->getNodeNumber();
    Jacobian(6*node,0) = 1.;
    Jacobian(6*node+1,1) = 1.;
    Jacobian(6*node+2,2) = -q(4*node+3);
    Jacobian(6*node+3,2) = q(4*node+2);
    Jacobian(RangeV(6*node+2,6*node+3),2) /= sqrt(q(4*node+2)*q(4*node+2)+q(4*node+3)*q(4*node+3));
    frame->setJacobianOfTranslation(R->evalOrientation()(RangeV(0, 2), RangeV(0, 1)) * Jacobian(RangeV(0, qSize - 1), RangeV(0, 1)).T(),j);
    frame->setJacobianOfRotation(R->evalOrientation()(RangeV(0, 2), RangeV(2, 2)) * Jacobian(RangeV(0, qSize - 1), RangeV(2, 2)).T(),j);
  }

  void FlexibleBody1s33ANCF::updateGyroscopicAccelerations(NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s33ANCF::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s33ANCF::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      FlexibleBody1s::init(stage, config);

      initialised = true;

      l0 = L/Elements;
      Vec g = R->getOrientation().T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for(int i=0;i<Elements;i++) {
        qElement.push_back(Vec(12,INIT,0.));
        uElement.push_back(Vec(12,INIT,0.));
        discretization.push_back(new FiniteElement1s33ANCF(l0, rho, E, G, A, I0, I1, I2, g));
        if((fabs(rc1) > epsroot) or (fabs(rc2) > epsroot))
          static_cast<FiniteElement1s33ANCF*>(discretization[i])->setCurlRadius(rc1,rc2);
      }
      initM();
    }
    else
      FlexibleBody1s::init(stage, config);
  }

  void FlexibleBody1s33ANCF::plot() {
    FlexibleBody1s::plot();
  }

  void FlexibleBody1s33ANCF::setNumberElements(int n){
    Elements = n;
    if(openStructure) {
      qSize = 6 * (n+1);
    } else {
      qSize = 6 *  n   ;
    }
    uSize[0] = qSize;
    uSize[1] = qSize;
    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody1s33ANCF::BuildElements() {
    for(int i=0;i<Elements;i++) {
      int n = 6 * i ;

      if(i<Elements-1 || openStructure==true) {
        qElement[i] << q(n,n+11);
        uElement[i] << u(n,n+11);
      }
      else { // last finite element and ring closure
        qElement[i](0,5) << q(n,n+5);
        uElement[i](0,5) << u(n,n+5);
        qElement[i](6,11) << q(0,5);
        uElement[i](6,11) << u(0,5);
      }
    }
    updEle = false;
  }

  void FlexibleBody1s33ANCF::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
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

  void FlexibleBody1s33ANCF::initM() {
    for (int i = 0; i < (int) discretization.size(); i++)
      static_cast<FiniteElement1s33ANCF*>(discretization[i])->initM(); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), M); // assemble
    for (int i = 0; i < (int) discretization.size(); i++) {
      int j = 6 * i;
      LLM(RangeV(j, j + 5)) = facLL(M(RangeV(j, j + 5)));
      if (openStructure && i == (int) discretization.size() - 1)
        LLM(RangeV(j + 6, j + 11)) = facLL(M(RangeV(j + 6, j + 11)));
    }
  }

  void FlexibleBody1s33ANCF::initInfo() {
    FlexibleBody1s::init(unknownStage, InitConfigSet());
    l0 = L/Elements;
    Vec g = Vec("[0.;0.;0.]");
    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s33ANCF(l0, rho, E, G, A, I0, I1, I2, g));
      qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
    }
  }

  void FlexibleBody1s33ANCF::initRelaxed(double alpha) {
    if(!initialised) {
      if(Elements==0)
        throw(new MBSimError("(FlexibleBody1s33ANCF::initRelaxed): Set number of finite elements!"));
      Vec q0Dummy(q0.size(),INIT,0.);
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(6*i+0,6*i+1) = direction*double(L/Elements*i);
          q0Dummy(6*i+3) = direction(0);
          q0Dummy(6*i+4) = direction(1);
        }
      }
      else {
        double R = L/(2*M_PI);

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(6*i+0) = R*cos(alpha_);
          q0Dummy(6*i+1) = R*sin(alpha_);
          q0Dummy(6*i+3) = -sin(alpha_);
          q0Dummy(6*i+4) = cos(alpha_);
        }
      }
      setq0(q0Dummy);
      setu0(Vec(q0Dummy.size(),INIT,0.));
    }
  }

  void FlexibleBody1s33ANCF::setCurlRadius(double rc1_, double rc2_) {
    rc1 = rc1_;
    rc2 = rc2_;
    if(initialised)
      for(int i = 0; i < Elements; i++)
        static_cast<FiniteElement1s33ANCF*>(discretization[i])->setCurlRadius(rc1,rc2);
  }

}
