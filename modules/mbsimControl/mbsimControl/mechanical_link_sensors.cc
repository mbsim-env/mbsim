/* Copyright (C) 2004-2020 MBSim Development Team
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
#include "mbsimControl/mechanical_link_sensors.h"
#include "mbsim/links/mechanical_link.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void MechanicalLinkSensor::init(InitStage stage, const InitConfigSet &config) {
    LinkSensor::init(stage, config);
    if(stage==resolveStringRef) {
      if(not dynamic_cast<MechanicalLink*>(link))
        throwError("Link is not a mechanichal link!");
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MechanicalLinkForceSensor)

  void MechanicalLinkForceSensor::updateSignal() {
    s = static_cast<MechanicalLink*>(link)->evalForce(i);
    upds = false;
  }

  void MechanicalLinkForceSensor::initializeUsingXML(DOMElement * element) {
    MechanicalLinkSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"forceNumber");
    if(e) setForceNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MechanicalLinkMomentSensor)

  void MechanicalLinkMomentSensor::updateSignal() {
    s = static_cast<MechanicalLink*>(link)->evalMoment(i);
    upds = false;
  }

  void MechanicalLinkMomentSensor::initializeUsingXML(DOMElement * element) {
    MechanicalLinkSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"momentNumber");
    if(e) setMomentNumber(E(e)->getText<Index>()-1);
  }

}
