/* Copyright (C) 2004-2018 MBSim Development Team
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
#include "mbsimControl/contact_sensor.h"
#include "mbsim/links/contact.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void ContactSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"contact");
    contactString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"singleContactNumber");
    if (e) setSingleContactNumber(E(e)->getText<Index>()-1);
  }

  void ContactSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (contactString!="")
        setContact(getByPath<Contact>(contactString));
      Sensor::init(stage, config);
    }
    else if (stage==unknownStage) {
      Sensor::init(stage, config);
      if(i<0 or i>=int(static_cast<Contact*>(contact)->getSubcontacts().size()))
        throwError("Single contact number out of range!");
    }
    else
      Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, GeneralizedRelativeContactPositionSensor)

  int GeneralizedRelativeContactPositionSensor::getSignalSize() const {
    return contact->getSingleContact(i).getGeneralizedRelativePositionSize();
  }

  void GeneralizedRelativeContactPositionSensor::updateSignal() {
    s = contact->getSingleContact(i).evalGeneralizedRelativePosition();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, GeneralizedRelativeContactVelocitySensor)

  int GeneralizedRelativeContactVelocitySensor::getSignalSize() const {
    return contact->getSingleContact(i).getGeneralizedRelativeVelocitySize();
  }

  void GeneralizedRelativeContactVelocitySensor::updateSignal() {
    s = contact->getSingleContact(i).evalGeneralizedRelativeVelocity();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, GeneralizedContactForceSensor)

  int GeneralizedContactForceSensor::getSignalSize() const {
    return contact->getSingleContact(i).getGeneralizedForceSize();
  }

  void GeneralizedContactForceSensor::updateSignal() {
    s = contact->getSingleContact(i).evalGeneralizedForce();
    upds = false;
  }

}
