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
#include "node_frame.h"
#include "mbsimFlexibleBody/node_based_body.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, NodeFrame)

  void NodeFrame::updatePositions() {
    setPosition(static_cast<NodeBasedBody*>(parent)->evalNodalPosition(node));
    setOrientation(static_cast<NodeBasedBody*>(parent)->getNodalOrientation(node));
    updPos = false;
  }

  void NodeFrame::updateVelocities() {
    setVelocity(static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(node));
    setAngularVelocity(static_cast<NodeBasedBody*>(parent)->getNodalAngularVelocity(node));
    updVel = false;
  }

  void NodeFrame::updateAccelerations() {
    setAcceleration(static_cast<NodeBasedBody*>(parent)->evalNodalAcceleration(node));
    setAngularAcceleration(static_cast<NodeBasedBody*>(parent)->getNodalAngularAcceleration(node));
    updAcc = true;
  }

  void NodeFrame::updateJacobians(int j) {
    setJacobianOfTranslation(static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(node,j));
    setJacobianOfRotation(static_cast<NodeBasedBody*>(parent)->getNodalJacobianOfRotation(node,j));
    updJac[j] = false;
  }

  void NodeFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfTranslation(static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(node));
    setGyroscopicAccelerationOfRotation(static_cast<NodeBasedBody*>(parent)->getNodalGyroscopicAccelerationOfRotation(node));
    updGA = false;
  }

  void NodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit)
      node = static_cast<NodeBasedBody*>(parent)->getNodeIndex(node);
    NodeBasedFrame::init(stage,config);
  }

  void NodeFrame::initializeUsingXML(DOMElement *element) {
    NodeBasedFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumber");
    setNodeNumber(E(e)->getText<int>());
  }

}
