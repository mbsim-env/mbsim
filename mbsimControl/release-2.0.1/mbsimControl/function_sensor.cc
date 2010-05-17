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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimControl/function_sensor.h"
#include "mbsimControl/objectfactory.h"

using namespace MBSim;

namespace MBSimControl {
      
  FunctionSensor::FunctionSensor(const std::string &name, Function1<fmatvec::Vec, double>* function_) : Sensor(name), function(function_) {
    y=(*function)(0);
  }

  void FunctionSensor::setFunction(Function1<fmatvec::Vec, double>* function_) {
    function=function_; 
    y=(*function)(0); 
  }
  void FunctionSensor::updateg(double t) {
    Sensor::updateg(t);
    y=(*function)(t); 
  }

  void FunctionSensor::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"function");
    function=MBSim::ObjectFactory::getInstance()->getInstance()->createFunction1_VS(e->FirstChildElement()); 
    function->initializeUsingXML(e->FirstChildElement());
    y=(*function)(0);
  }

}

