/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "ffr_interface_node_frame.h"
#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FfrInterfaceNodeFrame)

  void FfrInterfaceNodeFrame::updatePositions() {
    setOrientation(R->evalOrientation()*ARP*(Id+tilde(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqERel())));
    WrRP = R->evalOrientation()*(KrKP+Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqERel());
    setPosition(R->getPosition() + WrRP);
    updPos = false;
  }

  void FfrInterfaceNodeFrame::updateVelocities() {
    Womrel = R->evalOrientation()*(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel());
    Wvrel = R->getOrientation()*(Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel());
    setAngularVelocity(R->evalAngularVelocity() + Womrel);
    setVelocity(R->evalVelocity() + crossProduct(R->evalAngularVelocity(), evalGlobalRelativePosition()) + Wvrel);
    updVel = false;
  }

  void FfrInterfaceNodeFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration() + crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()) + R->evalOrientation()*(Psi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel()));
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(), crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(), evalGlobalRelativeVelocity()) + R->getOrientation()*(Phi*static_cast<GenericFlexibleFfrBody*>(parent)->evalqdERel()));
    updAcc = true;
  }

  void FfrInterfaceNodeFrame::updateJacobians(int j) {
    getJacobianOfRotation(j,false) = R->evalJacobianOfRotation(j);
    getJacobianOfRotation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes(),gethSize(j)-1),R->evalOrientation()*Psi);
    getJacobianOfTranslation(j,false) = R->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition())*R->getJacobianOfRotation(j);
    getJacobianOfTranslation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes(),gethSize(j)-1),R->getOrientation()*Phi);
    updJac[j] = false;
  }

  void FfrInterfaceNodeFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + crossProduct(R->evalAngularVelocity(),evalGlobalRelativeAngularVelocity()));
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + crossProduct(R->getAngularVelocity(),crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition())) + 2.*crossProduct(R->getAngularVelocity(),getGlobalRelativeVelocity()));
    updGA = false;
  }

  void FfrInterfaceNodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      for(int i=0; i<nodes.size(); i++)
        nodes(i) = static_cast<NodeBasedBody*>(parent)->getNodeIndex(nodes(i));
      if(weights.size()==0)
        weights.resize(nodes.size(),INIT,1.0);
      R = static_cast<GenericFlexibleFfrBody*>(parent)->getFrameK();
      KrKP = (weights(0)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0));
      ARP = weights(0)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativeOrientation(nodes(0));
      Phi = (weights(0)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(0));
      Psi = (weights(0)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfRotation(nodes(0));
      for(int i=1; i<nodes.size(); i++) {
        KrKP += (weights(i)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i));
        Phi += (weights(i)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i));
        Psi += (weights(i)/nodes.size())*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfRotation(nodes(i));
      }
    }
    NodeBasedFrame::init(stage,config);
  }

  void FfrInterfaceNodeFrame::initializeUsingXML(DOMElement *element) {
    NodeBasedFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    setNodeNumbers(E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"weightingFactors");
    if(e) setWeightingFactors(E(e)->getText<VecV>());
  }

}
