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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "node_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, NodeFrame)

  void NodeFrame::updatePositions() {
    static_cast<NodeBasedBody*>(parent)->updatePositions(this);
    updPos = false;
  }

  void NodeFrame::updateVelocities() {
    static_cast<NodeBasedBody*>(parent)->updateVelocities(this);
    updVel = false;
  }

  void NodeFrame::updateAccelerations() {
    static_cast<NodeBasedBody*>(parent)->updateAccelerations(this);
    updAcc = true;
  }

  void NodeFrame::updateJacobians(int j) {
    static_cast<NodeBasedBody*>(parent)->updateJacobians(this,j);
    updJac[j] = false;
  }

  void NodeFrame::updateGyroscopicAccelerations() {
    static_cast<NodeBasedBody*>(parent)->updateGyroscopicAccelerations(this);
    updGA = false;
  }

  void NodeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumber");
    setNodeNumber(E(e)->getText<int>()-1);
  }

}
