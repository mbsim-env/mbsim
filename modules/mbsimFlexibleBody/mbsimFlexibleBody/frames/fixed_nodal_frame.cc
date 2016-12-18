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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FixedNodalFrame)

  void FixedNodalFrame::init(InitStage stage) {
    if(stage==resize) {
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
          for(int i=0; i<3; i++)
            plotColumns.push_back("displ("+MBSim::numtostr(i)+")");
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

  const fmatvec::Vec3& FixedNodalFrame::evalGlobalRelativeAngularVelocity() {
    if(updVel) updateVelocities();
    return Womrel;
  }

  void FixedNodalFrame::updatePositions() {
    setOrientation(R->evalOrientation()*ARP*(E + tilde(Psi*q)));
    WrRP = R->getOrientation()*(RrRP + Phi*q);
    setPosition(R->getPosition() + WrRP);
    updPos = false;
  }

  void FixedNodalFrame::updateVelocities() {
    Womrel = R->evalOrientation()*(Psi*qd);
    Wvrel = R->getOrientation()*(Phi*qd);
    setAngularVelocity(R->evalAngularVelocity() + Womrel);
    setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition()) + Wvrel);
    updVel = false;
  }

  void FixedNodalFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration() +  crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()) + R->evalOrientation()*(Psi*qdd));
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(), crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(), Wvrel) +  R->getOrientation()*(Phi*qdd));
    updAcc = false;
  }

  void FixedNodalFrame::updateJacobians(int j) {
    Mat3xV Phi_ = Phi;
    if(K0F.size()) {
      MatVx3 PhigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PhigeoT.set(i,K0F[i]*q);
      Phi_ += PhigeoT.T();
    }
    Mat3xV Psi_ = Psi;
    if(K0M.size()) {
      MatVx3 PsigeoT(nq,NONINIT);
      for(int i=0; i<3; i++)
        PsigeoT.set(i,K0M[i]*q);
      Psi_ += PsigeoT.T();
    }
    setJacobianOfRotation(R->getJacobianOfRotation(j));
    getJacobianOfRotation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-nq,gethSize(j)-1),R->evalOrientation()*Psi_);
    setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition())*R->getJacobianOfRotation(j));
    getJacobianOfTranslation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-nq,gethSize(j)-1),R->getOrientation()*Phi_);
    updJac[j] = false;
  }

  void FixedNodalFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()));
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(),Wvrel));
    updGA = false;
  }

  void FixedNodalFrame::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(globalPosition)==enabled) {
      Vec3 u = Phi*q;
      if(K0F.size()) {
        MatVx3 PhigeoT(nq,NONINIT);
        for(int i=0; i<3; i++)
          PhigeoT.set(i,K0F[i]*q);
        u += PhigeoT*q;
      }
      for(int i=0; i<3; i++)
        plotVector.push_back(u(i));
      Vector<Fixed<6>,double> sigma = sigma0;
      if(sigmahel.cols()) {
        fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> sigmahe = sigmahel;
        for(unsigned int i=0; i<sigmahen.size(); i++)
          sigmahe += sigmahen[i]*q.e(i);
        sigma += sigmahe*q;
      }
      for(int i=0; i<6; i++)
        plotVector.push_back(sigma(i));
      }
      Frame::plot();
    }
  }

  void FixedNodalFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    DOMElement *ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeMatrixOfTranslation");
    setShapeMatrixOfTranslation(getMat(ec));
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeMatrixOfRotation");
    if(ec) setShapeMatrixOfRotation(getMat(ec));
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
      K0F = vector<SqrMatV>(3);
      ec=ec->getFirstElementChild();
      for(int i=0; i<3; i++) {
        K0F[i].resize() = getSqrMat(ec);
        ec=ec->getNextElementSibling();
      }
    }
    ec=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToMoment");
    if(ec) {
      K0M = vector<SqrMatV>(3);
      ec=ec->getFirstElementChild();
      for(int i=0; i<3; i++) {
        K0M[i].resize() = getSqrMat(ec);
        ec=ec->getNextElementSibling();
      }
    }
  }

}
