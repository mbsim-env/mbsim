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
#include "mbsimHydraulics/hydraulic_sensor.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/environment.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, FlowSensor)

  void FlowSensor::updateSignal() {
    s = line->evalQIn();
    upds = false;
  }

  void FlowSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"hline");
    lineString=E(e)->getAttribute("ref");
  }

  void FlowSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (lineString!="")
        setHLine(getByPath<HLine>(lineString));
      Sensor::init(stage, config);
    }
    else
      Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, PressureSensor)

  void PressureSensor::updateSignal() {
    s = node->evalGeneralizedForce();
    upds = false;
  }

  void PressureSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"hnode");
    nodeString=E(e)->getAttribute("ref");
  }

  void PressureSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (nodeString!="")
        setHNode(getByPath<HNode>(nodeString));
      Sensor::init(stage, config);
    }
    else if (stage==preInit) {
      addDependency(node);
      Sensor::init(stage, config);
    }
    else
      Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, TemperatureSensor)
  
  void TemperatureSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {
      s(0)=HydraulicEnvironment::getInstance()->getTemperature();
      Sensor::init(stage, config);
    }
    else
      Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, KinematicViscositySensor)
  
  void KinematicViscositySensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit) {
      s(0)=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      Sensor::init(stage, config);
    }
    else
      Sensor::init(stage, config);
  }

}
