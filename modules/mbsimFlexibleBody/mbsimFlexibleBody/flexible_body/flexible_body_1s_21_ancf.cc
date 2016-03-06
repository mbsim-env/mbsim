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
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"
#include <boost/swap.hpp>

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {


  FlexibleBody1s21ANCF::FlexibleBody1s21ANCF(const string &name, bool openStructure) : FlexibleBody1s(name,openStructure), Elements(0), l0(0), E(0), A(0), I(0), rho(0), rc(0.), deps(0.), dkappa(0.), initialised(false), v0(0.), Euler(false), sOld(-1e12) { }

  void FlexibleBody1s21ANCF::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 4 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloVec(j,j+7) += locVec;
    }
    else { // ring closure at finite element (end,1)
      gloVec(j,j+3) += locVec(0,3);
      gloVec(0,  3) += locVec(4,7);
    }
  }

  void FlexibleBody1s21ANCF::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 4 * n;

    if(n < Elements - 1 || openStructure==true) {
      gloMat(Index(j,j+7),Index(j,j+7)) += locMat;
    }
    else { // ring closure at finite element (end,1)
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

  void FlexibleBody1s21ANCF::updatePositions(double t, Frame1s *frame) {
    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement); // Lagrange parameter of affected FE
    frame->setPosition(R->getPosition(t) + R->getOrientation(t) *  static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->getPosition(getqElement(currentElement),sLocal));
    frame->setOrientation(R->getOrientation(t) *  static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->getOrientation(getqElement(currentElement),sLocal));
  }

  void FlexibleBody1s21ANCF::updateVelocities(double t, Frame1s *frame) {
    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(), sLocal, currentElement); // Lagrange parameter of affected FE
    frame->setVelocity(R->getOrientation(t) *  static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->getVelocity(getqElement(currentElement),getuElement(currentElement),sLocal));
    frame->setAngularVelocity(R->getOrientation(t) *  static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->getAngularVelocity(getqElement(currentElement),getuElement(currentElement),sLocal));
  }

  void FlexibleBody1s21ANCF::updateAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21ANCF::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s21ANCF::updateJacobians(double t, Frame1s *frame, int j) {
    Index All(0, 3 - 1);
    Mat Jacobian(qSize, 3, INIT, 0.);

    double sLocal;
    int currentElement;
    BuildElement(frame->getParameter(),sLocal,currentElement);
    Mat Jtmp = static_cast<FiniteElement1s21ANCF*>(discretization[currentElement])->JGeneralized(getqElement(currentElement),sLocal);
    if(currentElement<Elements-1 || openStructure) {
      Jacobian(Index(4*currentElement,4*currentElement+7),All) = Jtmp;
    }
    else { // ringstructure
      Jacobian(Index(4*currentElement,4*currentElement+3),All) = Jtmp(Index(0,3),All);
      Jacobian(Index(               0,                 3),All) = Jtmp(Index(4,7),All);
    }

    frame->setJacobianOfTranslation(R->getOrientation(t)(Index(0,2),Index(0,1))*Jacobian(Index(0,qSize-1),Index(0,1)).T(),j);
    frame->setJacobianOfRotation(R->getOrientation(t)(Index(0,2),Index(2,2))*Jacobian(Index(0,qSize-1),Index(2,2)).T(),j);
  }

  void FlexibleBody1s21ANCF::updateGyroscopicAccelerations(double t, Frame1s *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21ANCF::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s21ANCF::updatePositions(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    tmp(0) = q(4*node+0);
    tmp(1) = q(4*node+1);
    tmp(2) = 0.;
    frame->setPosition(R->getPosition(t) + R->getOrientation(t) * tmp);
    tmp(0) = q(4*node+2);
    tmp(1) = q(4*node+3);
    tmp(2) = 0.;
    tmp /= nrm2(tmp);
    frame->getOrientation(false).set(0, R->getOrientation() * tmp);
    tmp(0) = q(4*node+2);
    tmp(1) = -q(4*node+3);
    tmp(2) = 0.;
    tmp /= nrm2(tmp);
    boost::swap(tmp(0),tmp(1));
    frame->getOrientation(false).set(1, R->getOrientation() * tmp);
    frame->getOrientation(false).set(2, R->getOrientation().col(2));
  }

  void FlexibleBody1s21ANCF::updateVelocities(double t, NodeFrame *frame) {
    Vec3 tmp(NONINIT);
    int node = frame->getNodeNumber();
    tmp(0) = u(4*node+0);
    tmp(1) = u(4*node+1);
    tmp(2) = 0.;
    if(Euler) {
      tmp(0) += v0*q(4*node+2);
      tmp(1) += v0*q(4*node+3);
    }
    frame->setVelocity(R->getOrientation(t) * tmp);
    if(Euler) {
      double der2_1; // curvature first component
      double der2_2; // curvature second component
      if(4*(node+1)+1 < qSize) {
        der2_1 = 6./(l0*l0)*(q(4*(node+1))-q(4*(node)))-2./l0*(q(4*(node+1)+2)+2.*q(4*(node)+2));
        der2_2 = 6./(l0*l0)*(q(4*(node+1)+1)-q(4*(node)+1))-2./l0*(q(4*(node+1)+3)+2.*q(4*(node)+3));
      }
      else {
        der2_1 = 6./(l0*l0)*(q(4*(node-1))-q(4*(node)))+2./l0*(q(4*(node-1)+2)+2.*q(4*(node)+2));
        der2_2 = 6./(l0*l0)*(q(4*(node-1)+1)-q(4*(node)+1))+2./l0*(q(4*(node-1)+3)+2.*q(4*(node)+3));
      }
      tmp(0) = 0.; tmp(1) = 0.; tmp(2) = (-q(4*node+3)*(u(4*node+2) + v0*der2_1)+q(4*node+2)*(u(4*node+3) + v0*der2_2))/sqrt(q(4*node+2)*q(4*node+2)+q(4*node+3)*q(4*node+3));
    } else {
      tmp(0) = 0.;
      tmp(1) = 0.;
      tmp(2) = (-q(4*node+3)*u(4*node+2)+q(4*node+2)*u(4*node+3))/sqrt(q(4*node+2)*q(4*node+2)+q(4*node+3)*q(4*node+3));
    }
    frame->setAngularVelocity(R->getOrientation(t) * tmp);
  }

  void FlexibleBody1s21ANCF::updateAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21ANCF::updateAccelerations): Not implemented.");
  }

  void FlexibleBody1s21ANCF::updateJacobians(double t, NodeFrame *frame, int j) {
    Index All(0, 3 - 1);
    Mat Jacobian(qSize, 3, INIT, 0.);
    int node = frame->getNodeNumber();

    Jacobian(4*node,0) = 1.;
    Jacobian(4*node+1,1) = 1.;
    Jacobian(4*node+2,2) = -q(4*node+3);
    Jacobian(4*node+3,2) = q(4*node+2);
    Jacobian(Index(4*node+2,4*node+3),2) /= sqrt(q(4*node+2)*q(4*node+2)+q(4*node+3)*q(4*node+3));

    frame->setJacobianOfTranslation(R->getOrientation(t)(Index(0, 2), Index(0, 1)) * Jacobian(Index(0, qSize - 1), Index(0, 1)).T(),j);
    frame->setJacobianOfRotation(R->getOrientation(t)(Index(0, 2), Index(2, 2)) * Jacobian(Index(0, qSize - 1), Index(2, 2)).T(),j);
  }

  void FlexibleBody1s21ANCF::updateGyroscopicAccelerations(double t, NodeFrame *frame) {
    THROW_MBSIMERROR("(FlexibleBody1s21ANCF::updateGyroscopicAccelerations): Not implemented.");
  }

  void FlexibleBody1s21ANCF::init(InitStage stage) {
    if(stage==unknownStage) {
      FlexibleBody1s::init(stage);

      initialised = true;

      l0 = L/Elements;
      Vec g = R->getOrientation()(Index(0,2),Index(0,1)).T()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for(int i=0;i<Elements;i++) {
        qElement.push_back(Vec(8,INIT,0.));
        uElement.push_back(Vec(8,INIT,0.));
        discretization.push_back(new FiniteElement1s21ANCF(l0, A*rho, E*A, E*I, g, Euler, v0));
        if(fabs(rc) > epsroot())
          static_cast<FiniteElement1s21ANCF*>(discretization[i])->setCurlRadius(rc);
        static_cast<FiniteElement1s21ANCF*>(discretization[i])->setMaterialDamping(deps,dkappa);
      }
      initM();
    }
    else if(stage==plotting) {
      for(int i=0;i<q.size()/4;i++) {
        plotColumns.push_back("vel_abs node ("+numtostr(i)+")");
      }
      FlexibleBody1s::init(stage);
    }
    else
      FlexibleBody1s::init(stage);
  }

  void FlexibleBody1s21ANCF::plot(double t, double dt) {
    if(Euler) {
      for(int i=0;i<q.size()/4;i++) {
        plotVector.push_back(sqrt(pow(u(4*i+0) + v0*q(4*i+2),2.)+pow(u(4*i+1) + v0*q(4*i+3),2.)));
      }
    }
    else {
      for(int i=0;i<q.size()/4;i++) {
        plotVector.push_back(sqrt(pow(u(4*i+0),2.)+pow(u(4*i+1),2.)));
      }
    }
    FlexibleBody1s::plot(t,dt);
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
        uElement[i](4,7) << u(0,3);
      }
    }
    updEle = false;
  }

  void FlexibleBody1s21ANCF::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal,L);
    if(openStructure && sGlobal >= L) remainder += L; // remainder \in (-eps,L+eps)
    if(!openStructure && sGlobal < 0.) remainder += L; // remainder \in [0,L)

    currentElement = int(remainder/l0);
    sLocal = remainder - (currentElement) * l0; // Lagrange/Euler parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    // contact solver computes too large sGlobal at the end of the entire beam is not considered only for open structure
    // for closed structure even sGlobal < L (but sGlobal ~ L) values could lead - due to numerical problems - to a wrong currentElement computation
    if(currentElement >= Elements) {
      currentElement =  Elements-1;
      sLocal += l0;
    }
  }

  void FlexibleBody1s21ANCF::initM() {
    for (int i = 0; i < (int) discretization.size(); i++)
      static_cast<FiniteElement1s21ANCF*>(discretization[i])->initM(); // compute attributes of finite element
    for (int i = 0; i < (int) discretization.size(); i++)
      GlobalMatrixContribution(i, discretization[i]->getM(), M[0]); // assemble
    for (int i = 0; i < (int) discretization.size(); i++) {
      int j = 4 * i;
      LLM[0](Index(j, j + 3)) = facLL(M[0](Index(j, j + 3)));
      if (openStructure && i == (int) discretization.size() - 1)
        LLM[0](Index(j + 4, j + 7)) = facLL(M[0](Index(j + 4, j + 7)));
    }
  }

  void FlexibleBody1s21ANCF::initInfo() {
    FlexibleBody1s::init(unknownStage);
    l0 = L/Elements;
    Vec g = Vec("[0.;0.;0.]");
    for(int i=0;i<Elements;i++) {
      discretization.push_back(new FiniteElement1s21ANCF(l0, A*rho, E*A, E*I, g, Euler, v0));
      qElement.push_back(Vec(discretization[0]->getqSize(),INIT,0.));
      uElement.push_back(Vec(discretization[0]->getuSize(),INIT,0.));
    }
    BuildElements();
  }

  void FlexibleBody1s21ANCF::initRelaxed(double alpha) {
    if(!initialised) {
      if(Elements==0)
        throw(new MBSimError("(FlexibleBody1s21ANCF::initRelaxed): Set number of finite elements!"));
      Vec q0Dummy(q0.size(),INIT,0.);
      if(openStructure) {
        Vec direction(2);
        direction(0) = cos(alpha);
        direction(1) = sin(alpha);

        for(int i=0;i<=Elements;i++) {
          q0Dummy(4*i+0,4*i+1) = direction*double(L/Elements*i);
          q0Dummy(4*i+2) = direction(0);
          q0Dummy(4*i+3) = direction(1);
        }
      }
      else {
        double R = L/(2*M_PI);

        for(int i=0;i<Elements;i++) {
          double alpha_ = i*(2*M_PI)/Elements;
          q0Dummy(4*i+0) = R*cos(alpha_);
          q0Dummy(4*i+1) = R*sin(alpha_);
          q0Dummy(4*i+2) = -sin(alpha_);
          q0Dummy(4*i+3) = cos(alpha_);
        }
      }
      setq0(q0Dummy);
      setu0(Vec(q0Dummy.size(),INIT,0.));
    }
  }

  void FlexibleBody1s21ANCF::setCurlRadius(double rc_) {
    rc = rc_;
    if(initialised)
      for(int i = 0; i < Elements; i++)
        static_cast<FiniteElement1s21ANCF*>(discretization[i])->setCurlRadius(rc);
  }

  void FlexibleBody1s21ANCF::setMaterialDamping(double deps_, double dkappa_) {
    deps = deps_;
    dkappa = dkappa_;
    if(initialised)
      for(int i = 0; i < Elements; i++)
        static_cast<FiniteElement1s21ANCF*>(discretization[i])->setMaterialDamping(deps,dkappa);
  }

  void FlexibleBody1s21ANCF::setEulerPerspective(bool Euler_, double v0_) {
    if(openStructure) {
      throw(new MBSim::MBSimError("(FlexibleBody1s21ANCF::setEulerPerspective): implemented only for closed structures!"));
    }
    else {
      Euler = Euler_;
      v0 = v0_;
    }
  }

}

