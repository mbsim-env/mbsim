/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2012 Martin Förg

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   */

#include <config.h>
#include "sensor.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "basic_widgets.h"
#include "kinetics_widgets.h"
#include "extended_widgets.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Sensor::Sensor(const string &str, Element *parent) : Signal(str, parent) {
  }

  Sensor::~Sensor() {
  }

  GeneralizedCoordinateSensor::GeneralizedCoordinateSensor(const string &str, Element *parent) : Sensor(str, parent) {
    object.setProperty(new ObjectOfReferenceProperty("",this,MBSIMCONTROL%"object"));
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"), "", MBSIMCONTROL%"index"));
    index.setProperty(new ExtPhysicalVarProperty(input));
  }

  void GeneralizedCoordinateSensor::initialize() {
    Sensor::initialize();
    object.initialize();
  }

  void GeneralizedCoordinateSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    object.initializeUsingXML(element);
    index.initializeUsingXML(element);
  }

  DOMElement* GeneralizedCoordinateSensor::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Sensor::writeXMLFile(parent);
    object.writeXMLFile(ele0);
    index.writeXMLFile(ele0);
    return ele0;
  }

  AbsoluteCoordinateSensor::AbsoluteCoordinateSensor(const string &str, Element *parent) : Sensor(str, parent) {
    frame.setProperty(new FrameOfReferenceProperty("",this,MBSIMCONTROL%"frame"));
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMCONTROL%"direction"));
    direction.setProperty(new ExtPhysicalVarProperty(input));
  }

  void AbsoluteCoordinateSensor::initialize() {
    Sensor::initialize();
    frame.initialize();
  }

  void AbsoluteCoordinateSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    frame.initializeUsingXML(element);
    direction.initializeUsingXML(element);
  }

  DOMElement* AbsoluteCoordinateSensor::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Sensor::writeXMLFile(parent);
    frame.writeXMLFile(ele0);
    direction.writeXMLFile(ele0);
    return ele0;
  }

  FunctionSensor::FunctionSensor(const string &str, Element *parent) : Sensor(str, parent) {
    //  vector<Property*> property;
    //  property.push_back(new ConstantFunctionProperty);
    //  property.push_back(new LinearFunctionProperty);
    //  property.push_back(new QuadraticFunctionProperty);
    //  property.push_back(new SinusoidalFunctionProperty);
    //  property.push_back(new TabularFunctionProperty);
    //  property.push_back(new SummationFunctionProperty);
    //  vector<string> var;
    //  var.push_back("t");
    //  property.push_back(new SymbolicFunctionProperty("VS",var));
    function.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIMCONTROL%"function"));
  }

  void FunctionSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    function.initializeUsingXML(element);
  }

  DOMElement* FunctionSensor::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Sensor::writeXMLFile(parent);
    function.writeXMLFile(ele0);
    return ele0;
  }

  SignalProcessingSystemSensor::SignalProcessingSystemSensor(const string &str, Element *parent) : Sensor(str, parent) {
    spsRef.setProperty(new LinkOfReferenceProperty("",this,MBSIMCONTROL%"signalProcessingSystem"));
  }

  void SignalProcessingSystemSensor::initialize() {
    Sensor::initialize();
    spsRef.initialize();
  }

  void SignalProcessingSystemSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    spsRef.initializeUsingXML(element);
  }

  DOMElement* SignalProcessingSystemSensor::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Sensor::writeXMLFile(parent);
    spsRef.writeXMLFile(ele0);
    return ele0;
  }

}
