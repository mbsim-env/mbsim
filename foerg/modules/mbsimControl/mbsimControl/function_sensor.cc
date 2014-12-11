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
#include "mbsim/objectfactory.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionSensor, MBSIMCONTROL%"FunctionSensor")
      
  FunctionSensor::FunctionSensor(const std::string &name, MBSim::Function<VecV(double)>* function_) : Sensor(name), function(function_) {
    function->setParent(this);
    s=(*function)(0);
  }

  void FunctionSensor::setFunction(MBSim::Function<fmatvec::VecV(double)>* function_) {
    function=function_; 
    function->setParent(this);
    function->setName("Function");
    s=(*function)(0); 
  }

  void FunctionSensor::updateStateDependentVariables(double t) {
    Sensor::updateStateDependentVariables(t);
    s=(*function)(t); 
  }

  void FunctionSensor::updateg(double t) {
    throw;
    Sensor::updateg(t);
    s=(*function)(t); 
  }

  void FunctionSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    setFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<VecV(double)> >(e->getFirstElementChild())); 
  }

  void FunctionSensor::init(MBSim::Element::InitStage stage) {
    if(stage==preInit) {
      Signal::init(stage);
      addDependencies(function->getDependencies());
    }
    else
      Sensor::init(stage);
    function->init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function_SSEvaluation, MBSIMCONTROL%"Function_SSEvaluation")

  void Function_SSEvaluation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString = E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    setFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild())); 
  }

  void Function_SSEvaluation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signalString!="")
        setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else
      Signal::init(stage);
    fun->init(stage);
  }

  void Function_SSEvaluation::updateStateDependentVariables(double t) {
    VecV y=signal->getSignal();
    for (int i=0; i<y.size(); i++)
      y(i)=(*fun)(y(i));
    s = y;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function_SSSEvaluation, MBSIMCONTROL%"Function_SSSEvaluation")

  void Function_SSSEvaluation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"firstInputSignal");
    signal1String = E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"secondInputSignal");
    signal2String = E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"function");
    setFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double,double)> >(e->getFirstElementChild())); 
  }

  void Function_SSSEvaluation::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (signal1String!="")
        setSignals(getByPath<Signal>(signal1String), getByPath<Signal>(signal2String));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Signal::init(stage);
      addDependencies(fun->getDependencies());
    }
    else
      Signal::init(stage);
    fun->init(stage);
  }

  void Function_SSSEvaluation::updateStateDependentVariables(double t) {
    s = VecV(1, INIT, (*fun)(signal1->getSignal()(0), signal2->getSignal()(0)));
  }

}

