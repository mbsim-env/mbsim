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
#include "interface_node_frame.h"
#include "mbsimFlexibleBody/node_based_body.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, InterfaceNodeFrame)

  void InterfaceNodeFrame::updatePositions() {
    getPosition(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalPosition(nodes(0));
    for(int i=1; i<nodes.size(); i++)
      getPosition(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalPosition(nodes(i));
    setOrientation(static_cast<NodeBasedBody*>(parent)->getNodalOrientation(nodes(0)));
    updPos = false;
  }

  void InterfaceNodeFrame::updateVelocities() {
    getVelocity(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(nodes(0));
    getAngularVelocity(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalAngularVelocity(nodes(0));
    for(int i=1; i<nodes.size(); i++) {
      getVelocity(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(nodes(i));
      getAngularVelocity(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalAngularVelocity(nodes(i));
    }
    updVel = false;
  }

  void InterfaceNodeFrame::updateAccelerations() {
    getAcceleration(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalAcceleration(nodes(0));
    getAngularAcceleration(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalAngularAcceleration(nodes(0));
    for(int i=1; i<nodes.size(); i++) {
      getAcceleration(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalAcceleration(nodes(i));
      getAngularAcceleration(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalAngularAcceleration(nodes(i));
    }
    updAcc = true;
  }

  void InterfaceNodeFrame::updateJacobians(int j) {
    getJacobianOfTranslation(j,false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(nodes(0),j);
    getJacobianOfRotation(j,false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalJacobianOfRotation(nodes(0),j);
    for(int i=1; i<nodes.size(); i++) {
      getJacobianOfTranslation(j,false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(nodes(i),j);
      getJacobianOfRotation(j,false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalJacobianOfRotation(nodes(i),j);
    }
    updJac[j] = false;
  }

  void InterfaceNodeFrame::updateGyroscopicAccelerations() {
    getGyroscopicAccelerationOfTranslation(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(nodes(0));
    getGyroscopicAccelerationOfRotation(false) = (weights(0)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalGyroscopicAccelerationOfRotation(nodes(0));
    for(int i=1; i<nodes.size(); i++) {
      getGyroscopicAccelerationOfTranslation(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(nodes(i));
      getGyroscopicAccelerationOfRotation(false) += (weights(i)/sum)*static_cast<NodeBasedBody*>(parent)->getNodalGyroscopicAccelerationOfRotation(nodes(i));
    }
    updGA = false;
  }

  void InterfaceNodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      for(int i=0; i<nodes.size(); i++)
        nodes(i) = static_cast<NodeBasedBody*>(parent)->getNodeIndex(nodes(i));
      if(weights.size()==0)
        weights.resize(nodes.size(),INIT,1.0);
      sum = weights(0);
      for(int i=1; i<weights.size(); i++)
        sum += weights(i);
    }
    NodeBasedFrame::init(stage,config);
  }

  void InterfaceNodeFrame::initializeUsingXML(DOMElement *element) {
    NodeBasedFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    setNodeNumbers(E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"weightingFactors");
    if(e) setWeightingFactors(E(e)->getText<VecV>());
  }

}
