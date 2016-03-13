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

  void NodeFrame::updatePositions(double t) {
    static_cast<FlexibleBody*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void NodeFrame::updateVelocities(double t) {
    static_cast<FlexibleBody*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void NodeFrame::updateAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateAccelerations(t,this);
    updateAcc = true;
  }

  void NodeFrame::updateJacobians(double t, int j) {
    static_cast<FlexibleBody*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void NodeFrame::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

  void NodeFrame::updateAngles(double t) {
    static_cast<FlexibleBody*>(parent)->updateAngles(t,this);
    updAngles = false;
  }

  void NodeFrame::updateDerAngles(double t) {
    static_cast<FlexibleBody*>(parent)->updateDerAngles(t,this);
    updDerAngles = false;
  }

  void NodeFrame::resetUpToDate() {
    Frame::resetUpToDate();
    updAngles = true;
    updDerAngles = true;
  }

  void NodeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumber");
    setNodeNumber(getInt(e));
  }

  DOMElement* NodeFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMFLEX%"nodeNumber", getNodeNumber());
    return ele0;
  }

}
