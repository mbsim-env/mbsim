/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsimControl/object_sensors.h"
#include "mbsim/objects/object.h"
#include "mbsim/dynamic_system.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void ObjectSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"object");
    objectString=E(e)->getAttribute("ref");
  }

  void ObjectSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (objectString!="")
        setObject(getByPath<Object>(objectString));
    }
    Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, GeneralizedPositionSensor)

  int GeneralizedPositionSensor::getSignalSize() const {
    return object->getGeneralizedPositionSize();
  }

  void GeneralizedPositionSensor::updateSignal() {
    s = object->evalGeneralizedPosition();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, GeneralizedVelocitySensor)

  int GeneralizedVelocitySensor::getSignalSize() const {
    return object->getGeneralizedVelocitySize();
  }

  void GeneralizedVelocitySensor::updateSignal() {
    s = object->evalGeneralizedVelocity();
    upds = false;
  }

}
