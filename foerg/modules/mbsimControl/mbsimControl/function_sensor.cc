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
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/obsolet_hint.h"
#include "mbsimControl/defines.h"
#include "mbsim/objectfactory.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, FunctionSensor, MBSIMCONTROLNS"FunctionSensor")
      
  FunctionSensor::FunctionSensor(const std::string &name, MBSim::Function<VecV(double)>* function_) : Sensor(name), function(function_) {
    y=(*function)(0);
  }

  void FunctionSensor::setFunction(MBSim::Function<fmatvec::VecV(double)>* function_) {
    function=function_; 
    y=(*function)(0); 
  }
  void FunctionSensor::updateg(double t) {
    Sensor::updateg(t);
    y=(*function)(t); 
  }

  void FunctionSensor::initializeUsingXML(TiXmlElement *element) {
    Sensor::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"function");
    function=MBSim::ObjectFactory<MBSim::Function<VecV(double)> >::create<MBSim::Function<VecV(double)> >(e->FirstChildElement()); 
    function->initializeUsingXML(e->FirstChildElement());
    y=(*function)(0);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Function_SSEvaluation, MBSIMCONTROLNS"Function_SSEvaluation")

  void Function_SSEvaluation::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    signalString = e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"function");
    fun=MBSim::ObjectFactory<MBSim::Function<double(double)> >::create<MBSim::Function<double(double)> >(e->FirstChildElement()); 
    fun->initializeUsingXML(e->FirstChildElement());
  }

  void Function_SSEvaluation::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(process_signal_string(signalString)));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec Function_SSEvaluation::getSignal() {
    Vec y=signal->getSignal().copy();
    for (int i=0; i<y.size(); i++)
      y(i)=(*fun)(y(i));
    return y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Function_SSSEvaluation, MBSIMCONTROLNS"Function_SSSEvaluation")

  void Function_SSSEvaluation::initializeUsingXML(TiXmlElement *element) {
    Signal::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"firstInputSignal");
    signal1String = e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"secondInputSignal");
    signal2String = e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"function");
    fun=MBSim::ObjectFactory<MBSim::Function<double(double,double)> >::create<MBSim::Function<double(double,double)> >(e->FirstChildElement()); 
    fun->initializeUsingXML(e->FirstChildElement());
  }

  void Function_SSSEvaluation::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (signal1String!="")
        setSignals(getByPath<Signal>(process_signal_string(signal1String)), getByPath<Signal>(process_signal_string(signal2String)));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
  }

  Vec Function_SSSEvaluation::getSignal() {
    return Vec(1, INIT, (*fun)(signal1->getSignal()(0), signal2->getSignal()(0)));
  }

}

