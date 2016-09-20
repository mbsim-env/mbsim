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
#include "mbsimFlexibleBody/namespace.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FixedNodalFrame::init(InitStage stage) {
    if(stage==resize) {
      if(not(Phi.cols()))
        Phi.resize(nq);
      if(not(Psi.cols()))
        Psi.resize(nq);
      q.resize(nq);
      qd.resize(nq);
      qdd.resize(nq);
      WJD[0].resize(nq,hSize[0]);
      WJD[1].resize(nq,hSize[1]);
      for(int i=0; i<nq; i++) {
        WJD[0](i,hSize[0]-nq+i) = 1;
        WJD[1](i,hSize[1]-nq+i) = 1;
      }
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
      
  const Vec3& FixedNodalFrame::evalGlobalRelativePosition() {
    if(updPos) updatePositions();
    return WrRP;
  }

  const Mat3xV& FixedNodalFrame::evalGlobalPhi() {
    if(updPos) updatePositions();
    return WPhi;
  }

  const Mat3xV& FixedNodalFrame::evalGlobalPsi() {
    if(updPos) updatePositions();
    return WPsi;
  }

  void FixedNodalFrame::updatePositions() {
    APK = E+tilde(Psi*q);
    setOrientation(R->evalOrientation()*ARP*APK);
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
    updPos = false;
  }

  void FixedNodalFrame::updateVelocities() {
    setAngularVelocity(R->evalAngularVelocity() + evalGlobalPsi()*qd);
    setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), getGlobalRelativePosition()) + getGlobalPhi()*qd);
    updVel = false;
  }

  void FixedNodalFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration() +  crossProduct(R->evalAngularVelocity(),evalGlobalPsi()*qd) + evalGlobalPsi()*qdd);
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), getGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(), crossProduct(R->getAngularVelocity(), getGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(),getGlobalPhi()*qd) + getGlobalPhi()*qdd);
    updAcc = false;
  }

  void FixedNodalFrame::updateJacobians(int j) {
    setJacobianOfRotation(R->getJacobianOfRotation(j) + evalGlobalPsi()*WJD[j],j);
    setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tilde(getGlobalRelativePosition())*R->getJacobianOfRotation(j) + getGlobalPhi()*WJD[j],j);
    updJac[j] = false;
  }

  void FixedNodalFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + crossProduct(R->evalAngularVelocity(),evalGlobalPsi()*qd));
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),getGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),getGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(),getGlobalPhi()*qd));
    updGA = false;
  }

  void FixedNodalFrame::plot() {
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
      Frame::plot();
    }
  }

  void FixedNodalFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    DOMElement *ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modeShapeOfTranslation");
    setModeShapeOfTranslation(getMat(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modeShapeOfRotation");
    if(ec) setModeShapeOfRotation(getSqrMat(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"stressMatrix");
    if(ec) setStressMatrix(getMat(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStressMatrix");
    if(ec) {
      ec=ec->getFirstElementChild();
      while(ec) {
        sigmahen.push_back(getMat(ec));
        ec=ec->getNextElementSibling();
      }
    }
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"initialStress");
    if(ec) setInitialStress(getVec(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToForce");
    if(ec) {
      ec=ec->getFirstElementChild();
      while(ec) {
        K0F.push_back(getSqrMat(ec));
        ec=ec->getNextElementSibling();
      }
    }
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToMoment");
    if(ec) {
      ec=ec->getFirstElementChild();
      while(ec) {
        K0M.push_back(getSqrMat(ec));
        ec=ec->getNextElementSibling();
      }
    }
  }

  DOMElement* FixedNodalFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
   return ele0;
  }

}
