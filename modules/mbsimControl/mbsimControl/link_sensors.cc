/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimControl/link_sensors.h"

#include "mbsim/links/link.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void LinkSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"link");
    linkString=E(e)->getAttribute("ref");
  }

  void LinkSensor::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (linkString!="")
        setLink(getByPath<Link>(linkString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinkDistanceSensor, MBSIMCONTROL%"GeneralizedRelativePositionSensor")

  void LinkDistanceSensor::updateSignal(double t) {
    s = link->getGeneralizedRelativePosition(t);
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinkVelocitySensor, MBSIMCONTROL%"GeneralizedRelativeVelocitySensor")

  void LinkVelocitySensor::updateSignal(double t) {
    s = link->getGeneralizedRelativeVelocity(t);
    upds = false;
  }

}
