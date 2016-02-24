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
#include "frame_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void Frame1s::updatePositions(double t) {
    static_cast<FlexibleBody1s*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void Frame1s::updateVelocities(double t) {
    static_cast<FlexibleBody1s*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void Frame1s::updateAccelerations(double t) {
    static_cast<FlexibleBody1s*>(parent)->updateAccelerations(t,this);
    updateAcc = true;
  }

  void Frame1s::updateJacobians(double t, int j) {
    static_cast<FlexibleBody1s*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void Frame1s::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody1s*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

  void Frame1s::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"parameter");
    setParameter(getDouble(e));
  }

  DOMElement* Frame1s::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMFLEX%"parameter", getParameter());
    return ele0;
  }

}
