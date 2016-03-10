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
#include "frame_2s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_2s.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void Frame2s::updatePositions(double t) {
    static_cast<FlexibleBody2s*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void Frame2s::updateVelocities(double t) {
    static_cast<FlexibleBody2s*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void Frame2s::updateAccelerations(double t) {
    static_cast<FlexibleBody2s*>(parent)->updateAccelerations(t,this);
    updateAcc = true;
  }

  void Frame2s::updateJacobians(double t, int j) {
    static_cast<FlexibleBody2s*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void Frame2s::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody2s*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

  void Frame2s::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"parameters");
    setParameters(getVec(e));
  }

  DOMElement* Frame2s::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMFLEX%"parameters", getParameters());
    return ele0;
  }

}
