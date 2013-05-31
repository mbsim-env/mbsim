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

using namespace std;
using namespace MBXMLUtils;

Sensor::Sensor(const string &str, Element *parent) : Signal(str, parent) {
}

Sensor::~Sensor() {
}

GeneralizedCoordinateSensor::GeneralizedCoordinateSensor(const string &str, Element *parent) : Sensor(str, parent) {
  object.setProperty(new ObjectOfReferenceProperty("",this,MBSIMCONTROLNS"object"));
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "", MBSIMCONTROLNS"index"));
  index.setProperty(new ExtPhysicalVarProperty(input));
}

void GeneralizedCoordinateSensor::initialize() {
  Sensor::initialize();
  object.initialize();
}

void GeneralizedCoordinateSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  object.initializeUsingXML(element);
  index.initializeUsingXML(element);
}

TiXmlElement* GeneralizedCoordinateSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  object.writeXMLFile(ele0);
  index.writeXMLFile(ele0);
  return ele0;
}

AbsoluteCoordinateSensor::AbsoluteCoordinateSensor(const string &str, Element *parent) : Sensor(str, parent) {
  frame.setProperty(new FrameOfReferenceProperty("",this,MBSIMCONTROLNS"frame"));
  direction.setProperty(new GeneralizedForceDirectionProperty(MBSIMCONTROLNS"direction"));
}

void AbsoluteCoordinateSensor::initialize() {
  Sensor::initialize();
  frame.initialize();
}

void AbsoluteCoordinateSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  frame.initializeUsingXML(element);
  direction.initializeUsingXML(element);
}

TiXmlElement* AbsoluteCoordinateSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  frame.writeXMLFile(ele0);
  direction.writeXMLFile(ele0);
  return ele0;
}

FunctionSensor::FunctionSensor(const string &str, Element *parent) : Sensor(str, parent) {
  vector<Property*> property;
  property.push_back(new ConstantFunction1Property("VS"));
  property.push_back(new QuadraticFunction1Property);
  property.push_back(new SinusFunction1Property);
  property.push_back(new TabularFunction1Property);
  property.push_back(new SummationFunction1Property);
  property.push_back(new SymbolicFunction1Property("VS"));
  function.setProperty(new GeneralChoiceProperty(MBSIMCONTROLNS"function",property));
}

void FunctionSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  function.initializeUsingXML(element);
}

TiXmlElement* FunctionSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  function.writeXMLFile(ele0);
  return ele0;
}

SignalProcessingSystemSensor::SignalProcessingSystemSensor(const string &str, Element *parent) : Sensor(str, parent) {
  spsRef.setProperty(new ExtraDynamicOfReferenceProperty("",this,MBSIMCONTROLNS"signalProcessingSystem"));
}

void SignalProcessingSystemSensor::initialize() {
  Sensor::initialize();
  spsRef.initialize();
}

void SignalProcessingSystemSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  spsRef.initializeUsingXML(element);
}

TiXmlElement* SignalProcessingSystemSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  spsRef.writeXMLFile(ele0);
  return ele0;
}
