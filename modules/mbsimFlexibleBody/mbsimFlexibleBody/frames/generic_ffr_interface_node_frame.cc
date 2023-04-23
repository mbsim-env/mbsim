/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "generic_ffr_interface_node_frame.h"
#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void GenericFfrInterfaceNodeFrame::updatePositions() {
    setOrientation(R->evalOrientation()*ARP*(Id+tilde(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqERel())));
    WrRP = R->evalOrientation()*(KrKP+Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqERel());
    setPosition(R->getPosition() + WrRP);
    updPos = false;
  }

  void GenericFfrInterfaceNodeFrame::updateVelocities() {
    Womrel = R->evalOrientation()*(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel());
    Wvrel = R->getOrientation()*(Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel());
    setAngularVelocity(R->evalAngularVelocity() + Womrel);
    setVelocity(R->evalVelocity() + crossProduct(R->evalAngularVelocity(), evalGlobalRelativePosition()) + Wvrel);
    updVel = false;
  }

  void GenericFfrInterfaceNodeFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration() + crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()) + R->evalOrientation()*(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel()));
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(), crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(), evalGlobalRelativeVelocity()) + R->getOrientation()*(Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel()));
    updAcc = true;
  }

  void GenericFfrInterfaceNodeFrame::updateJacobians(int j) {
    getJacobianOfRotation(j,false) = R->evalJacobianOfRotation(j);
    getJacobianOfRotation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes(),gethSize(j)-1),R->evalOrientation()*Psi);
    getJacobianOfTranslation(j,false) = R->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition())*R->getJacobianOfRotation(j);
    getJacobianOfTranslation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes(),gethSize(j)-1),R->getOrientation()*Phi);
    updJac[j] = false;
  }

  void GenericFfrInterfaceNodeFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()));
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(),getGlobalRelativeVelocity()));
    updGA = false;
  }

  void GenericFfrInterfaceNodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      double sum = 0;
      for(int i=0; i<nodes.size(); i++) {
        nodes(i) = static_cast<NodeBasedBody*>(parent)->getNodeIndex(nodes(i));
        sum += weights(i);
      }
      R = static_cast<GenericFlexibleFfrBody*>(parent)->getFrameK();
      ARP = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativeOrientation(nodes(0));
      Phi.resize(static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes());
      Psi.resize(static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes());
      for(int i=0; i<nodes.size(); i++) {
        KrKP += (weights(i)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i));
        Phi += (weights(i)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i));
      }
      if(approximateShapeMatrixOfRotation) {
	SymMat3 A;
	Mat3xV B(Phi.cols());
	for(int i=0; i<nodes.size(); i++) {
	  SqrMat3 tr = tilde(static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))-KrKP);
	  A += (weights(i)/sum)*tr.T()*tr;
	  B += (weights(i)/sum)*(tr.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i)));
	}
	Psi = -slvLL(A,B);
      }
      else {
        for(int i=0; i<nodes.size(); i++)
          Psi += (weights(i)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfRotation(nodes(i));
      }
    }
    NodeBasedFrame::init(stage,config);
  }

  void GenericFfrInterfaceNodeFrame::initializeUsingXML(DOMElement *element) {
    NodeBasedFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"approximateShapeMatrixOfRotation");
    if(e) setApproximateShapeMatrixOfRotation(E(e)->getText<bool>());
  }

}
