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
      qdd.resize(nq);
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
      
  const Vec3& FixedNodalFrame::getGlobalRelativePosition(double t) {
    if(updatePos) updatePositions(t);
    return WrRP;
  }

  const Mat3xV& FixedNodalFrame::getGlobalPhi(double t) {
    if(updatePos) updatePositions(t);
    return WPhi;
  }

  const Mat3xV& FixedNodalFrame::getGlobalPsi(double t) {
    if(updatePos) updatePositions(t);
    return WPsi;
  }

  void FixedNodalFrame::updatePositions(double t) {
    APK = E+tilde(Psi*q);
    setOrientation(R->getOrientation(t)*ARP*APK);
    if(K0F.size()) {
      MatVx3 PhigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PhigeoT.set(i,K0F[i]*q);
      WPhi = R->getOrientation()*(Phi + PhigeoT.T()); 
    }
    else
      WPhi = R->getOrientation()*Phi;
    if(K0M.size()) {
      MatVx3 PsigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PsigeoT.set(i,K0M[i]*q);
      WPsi = R->getOrientation()*(Psi + PsigeoT.T()); 
    }
    else
      WPsi = R->getOrientation()*Psi;
    WrRP = R->getOrientation()*RrRP + WPhi*q;
    setPosition(R->getPosition() + WrRP);
    updatePos = false;
  }

  void FixedNodalFrame::updateVelocities(double t) {
    setAngularVelocity(R->getAngularVelocity(t) + getGlobalPsi(t)*qd);
    setVelocity(R->getVelocity(t) + crossProduct(R->getAngularVelocity(), getGlobalRelativePosition(t)) + getGlobalPhi(t)*qd);
    updateVel = false;
  }

  void FixedNodalFrame::updateAccelerations(double t) {
    setAngularAcceleration(R->getAngularAcceleration(t) +  crossProduct(R->getAngularVelocity(t),getGlobalPsi(t)*qd) + getGlobalPsi(t)*qdd);
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), getGlobalRelativePosition(t)) + crossProduct(R->getAngularVelocity(), crossProduct(R->getAngularVelocity(), getGlobalRelativePosition(t))) + 2.*crossProduct(R->getAngularVelocity(),getGlobalPhi()*qd) + getGlobalPhi()*qdd);
    updateAcc = false;
  }

  void FixedNodalFrame::updateJacobians(double t, int j) {
    setJacobianOfTranslation(R->getJacobianOfTranslation(t,j) - tilde(getGlobalRelativePosition(t))*R->getJacobianOfRotation(t,j) + getGlobalPhi(t)*R->getJacobianOfDeformation(t,j),j);
    setJacobianOfRotation(R->getJacobianOfRotation(j) + getGlobalPsi()*R->getJacobianOfDeformation(j),j);
    setJacobianOfDeformation(R->getJacobianOfDeformation(j),j);
    updateJac[j] = false;
  }

  void FixedNodalFrame::updateGyroscopicAccelerations(double t) {
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation(t) + crossProduct(R->getGyroscopicAccelerationOfRotation(t),getGlobalRelativePosition(t)) + crossProduct(R->getAngularVelocity(t),crossProduct(R->getAngularVelocity(t),getGlobalRelativePosition(t))) + 2.*crossProduct(R->getAngularVelocity(t),getGlobalPhi(t)*qd));
    setGyroscopicAccelerationOfRotation(R->getGyroscopicAccelerationOfRotation() + crossProduct(R->getAngularVelocity(),getGlobalPsi()*qd));
    updateGA = false;
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

