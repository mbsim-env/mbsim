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
#include "mbsim/object.h"
#include "mbsim/dynamic_system.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void GeneralizedCoordinateSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"object");
    objectString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"index");
    index=getInt(e);
  }

  void GeneralizedCoordinateSensor::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (objectString!="")
        setObject(getByPath<Object>(objectString));
      Sensor::init(stage);
    }
    else if(stage==resize) {
      Sensor::init(stage);
      s.resize(1);
    }
    else
      Sensor::init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedPositionSensor, MBSIMCONTROL%"GeneralizedPositionSensor")

  void GeneralizedPositionSensor::updateh(double t, int j) {
    if (object->getq().size()!=0)
      s = ((object->getq()))(fmatvec::Index(index, index));
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedVelocitySensor, MBSIMCONTROL%"GeneralizedVelocitySensor")

  void GeneralizedVelocitySensor::updateh(double t, int j) {
    if (object->getu().size()==0)
      //return object->getu0()(fmatvec::Index(index, index));
      s = fmatvec::VecV(1, fmatvec::INIT, 0);
    else
      s = ((object->getu()))(fmatvec::Index(index, index));
  }

}