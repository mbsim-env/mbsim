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
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/obsolet_hint.h"

using namespace std;
using namespace fmatvec;

namespace MBSimHydraulics {

  Vec FlowSensor::getSignal() {
    return line->getQIn(); 
  }

  void FlowSensor::initializeUsingXML(TiXmlElement * element) {
    Sensor::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"hline");
    lineString=e->Attribute("ref");
  }

  void FlowSensor::init(MBSim::InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (lineString!="")
        setHLine(getByPath<HLine>(process_hline_string(lineString)));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  Vec PressureSensor::getSignal() {
    return node->getla(); 
  }

  void PressureSensor::initializeUsingXML(TiXmlElement * element) {
    Sensor::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"hnode");
    nodeString=e->Attribute("ref");
  }

  void PressureSensor::init(MBSim::InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (nodeString!="")
        setHNode(getByPath<HNode>(nodeString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  
  void TemperatureSensor::init(MBSim::InitStage stage) {
    if (stage==MBSim::preInit) {
      T(0)=HydraulicEnvironment::getInstance()->getTemperature();
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  
  void KinematicViscositySensor::init(MBSim::InitStage stage) {
    if (stage==MBSim::preInit) {
      nu(0)=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

}
