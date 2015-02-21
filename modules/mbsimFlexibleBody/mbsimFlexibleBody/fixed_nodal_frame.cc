/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "fixed_nodal_frame.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FixedNodalFrame::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<FixedNodalFrame>(saved_frameOfReference));
      Frame::init(stage);
    }
    else if(stage==resize) {
      WJD[0].resize(nq,hSize[0]);
      WJD[1].resize(nq,hSize[1]);
      if(not(Phi.cols()))
        Phi.resize(nq);
      if(not(Psi.cols()))
        Psi.resize(nq);
      q.resize(nq);
      qd.resize(nq);
      Frame::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(globalPosition)==enabled) {
          for(int i=0; i<6; i++)
            plotColumns.push_back("sigma("+MBSim::numtostr(i)+")");
        }
        Frame::init(stage);
      }
    }
    else
      Frame::init(stage);
  }
      
  void FixedNodalFrame::updateRelativePosition() {
    WPhi = R->getOrientation()*Phi; 
    WPsi = R->getOrientation()*Psi; 
    WrRP = R->getOrientation()*RrRP + WPhi*q; 
  }

  void FixedNodalFrame::updateRelativeOrientation() {
    APK = E+tilde(Psi*q); 
  }

  void FixedNodalFrame::updatePosition() {
    updateRelativePosition(); 
    setPosition(R->getPosition() + WrRP); 
  }

  void FixedNodalFrame::updateOrientation() {
    updateRelativeOrientation(); 
    setOrientation(R->getOrientation()*ARP*APK); 
  }

  void FixedNodalFrame::updateVelocity() {
    setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), WrRP) + WPhi*qd); 
  } 

  void FixedNodalFrame::updateAngularVelocity() {
    setAngularVelocity(R->getAngularVelocity() + WPsi*qd); 
  }

  void FixedNodalFrame::updateStateDependentVariables() {
    updatePosition();
    updateOrientation();
    updateVelocity();
    updateAngularVelocity();
  }

  void FixedNodalFrame::updateJacobians(int j) {
    if(K0F.size()) {
      MatVx3 PhigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PhigeoT.set(i,K0F[i]*q);
      WPhi = R->getOrientation()*(Phi + PhigeoT.T()); 
    }
    if(K0M.size()) {
      MatVx3 PsigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PsigeoT.set(i,K0M[i]*q);
      WPsi = R->getOrientation()*(Psi + PsigeoT.T()); 
    }
    fmatvec::SqrMat3 tWrRP = tilde(WrRP);
    setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tWrRP*R->getJacobianOfRotation(j) + WPhi*R->getJacobianOfDeformation(j),j);
    setJacobianOfRotation(R->getJacobianOfRotation(j) + WPsi*R->getJacobianOfDeformation(j),j);
    setJacobianOfDeformation(R->getJacobianOfDeformation(j),j);
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(j) - tWrRP*R->getGyroscopicAccelerationOfRotation(j) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),WrRP)) + 2.*crossProduct(R->getAngularVelocity(),WPhi*qd),j);
    setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation(j) + crossProduct(R->getAngularVelocity(),WPsi*qd),j);
  }

  void FixedNodalFrame::updateStateDerivativeDependentVariables(const fmatvec::Vec &ud) { 
    setAcceleration(getJacobianOfTranslation()*ud + getGyroscopicAccelerationOfTranslation()); 
    setAngularAcceleration(getJacobianOfRotation()*ud + getGyroscopicAccelerationOfRotation());
  }

  void FixedNodalFrame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
//      if(getPlotFeature(stresses)==enabled) {
      Vector<Fixed<6>,double> sigma = sigma0;
      if(sigmahel.cols()) {
        fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> sigmahe = sigmahel;
        for(unsigned int i=0; i<sigmahen.size(); i++)
          sigmahe += sigmahen[i]*q.e(i);
        sigma += sigmahe*q;
      }
      for(int i=0; i<6; i++)
        plotVector.push_back(sigma(i));
//      }
      Frame::plot(t,dt);
    }
  }

  void FixedNodalFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
//    DOMElement *ec=element->getFirstElementChild();
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
//    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativePosition");
//    if(ec) setRelativePosition(getVec3(ec));
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativeOrientation");
//    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  DOMElement* FixedNodalFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//     if(getFrameOfReference()) {
//        DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//        string str = string("../Frame[") + getFrameOfReference()->getName() + "]";
//        ele1->SetAttribute("ref", str);
//        ele0->LinkEndChild(ele1);
//      }
//     addElementText(ele0,MBSIM%"relativePosition",getRelativePosition());
//     addElementText(ele0,MBSIM%"relativeOrientation",getRelativeOrientation());
   return ele0;
  }

}

