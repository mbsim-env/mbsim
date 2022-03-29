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
      for(int i=0; i<nodes.size(); i++)
        nodes(i) = static_cast<NodeBasedBody*>(parent)->getNodeIndex(nodes(i));
      double sum = weights(0);
      for(int i=1; i<weights.size(); i++)
        sum += weights(i);
      R = static_cast<GenericFlexibleFfrBody*>(parent)->getFrameK();
      KrKP = (weights(0)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0));
      ARP = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativeOrientation(nodes(0));
      Phi <<= (weights(0)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(0));
      for(int i=1; i<nodes.size(); i++) {
        KrKP += (weights(i)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i));
        Phi += (weights(i)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i));
      }
      if(approximateShapeMatrixOfRotation) {
        int ne = static_cast<GenericFlexibleFfrBody*>(parent)->getNumberOfModeShapes();
        Psi.resize(ne,NONINIT);
        Vec3 al(NONINIT);
        al(0) = 0;
        al(1) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(2)+KrKP(2);
        al(2) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(1)-KrKP(1);
        double alq = (al.T()*al);
        if(alq <= 1e-8)
          throwError("Leverarm to small");
        Psi.set(RangeV(0,0),RangeV(0,ne-1), (weights(0)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(0))/alq);
        for(int i=1; i<nodes.size(); i++) {
          al(1) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(2)+KrKP(2);
          al(2) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(1)-KrKP(1);
          alq = (al.T()*al);
          if(alq <= 1e-8)
            throwError("Leverarm to small");
          Psi.add(RangeV(0,0),RangeV(0,ne-1), (weights(i)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i))/alq);
        }
        al(0) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(2)-KrKP(2);
        al(1) = 0;
        al(2) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(0)+KrKP(0);
        alq = (al.T()*al);
        if(alq <= 1e-8)
          throwError("Leverarm to small");
        Psi.set(RangeV(1,1),RangeV(0,ne-1), (weights(0)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(0))/alq);
        for(int i=1; i<nodes.size(); i++) {
          al(0) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(2)-KrKP(2);
          al(2) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(0)+KrKP(0);
          alq = (al.T()*al);
          if(alq <= 1e-8)
            throwError("Leverarm to small");
          Psi.add(RangeV(1,1),RangeV(0,ne-1), (weights(i)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i))/alq);
        }
        al(0) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(1)+KrKP(1);
        al(1) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(0))(0)-KrKP(0);
        al(2) = 0;
        alq = (al.T()*al);
        if(alq <= 1e-8)
          throwError("Leverarm to small");
        Psi.set(RangeV(2,2),RangeV(0,ne-1), (weights(0)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(0))/alq);
        for(int i=1; i<nodes.size(); i++) {
          al(0) = -static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(1)+KrKP(1);
          al(1) = static_cast<GenericFlexibleFfrBody*>(parent)->getNodalRelativePosition(nodes(i))(0)-KrKP(0);
          alq = (al.T()*al);
          if(alq <= 1e-8)
            throwError("Leverarm to small");
          Psi.add(RangeV(2,2),RangeV(0,ne-1), (weights(i)/sum)*al.T()*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfTranslation(nodes(i))/alq);
        }
      }
      else {
        Psi <<= (weights(0)/sum)*static_cast<GenericFlexibleFfrBody*>(parent)->getNodalShapeMatrixOfRotation(nodes(0));
        for(int i=1; i<nodes.size(); i++)
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
