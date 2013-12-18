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
#include "signal_.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "basic_widgets.h"
#include "kinetics_widgets.h"
#include "extended_widgets.h"
#include "property.h"

using namespace std;
using namespace MBXMLUtils;

Signal::Signal(const string &str, Element *parent) : Link(str, parent) {
}

Signal::~Signal() {
}

SignalAddition::SignalAddition(const string &str, Element *parent) : Signal(str, parent) {
  signalReferences.setProperty(new SignalReferencesProperty(this,""));
}

void SignalAddition::initialize() {
  Signal::initialize();

  signalReferences.initialize();
}

void SignalAddition::initializeUsingXML(TiXmlElement *element) {
  Signal::initializeUsingXML(element);
  signalReferences.initializeUsingXML(element);
}

TiXmlElement* SignalAddition::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Signal::writeXMLFile(parent);
  signalReferences.writeXMLFile(ele0);
  return ele0;
}

PIDController::PIDController(const string &str, Element *parent) : Signal(str, parent) {
  sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"inputSignal"));
  sdRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"derivativeOfInputSignal"));
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROLNS"P"));
  P.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"I"));
  I.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"D"));
  D.setProperty(new ExtPhysicalVarProperty(input));

}

void PIDController::initialize() {
  Signal::initialize();
  sRef.initialize();
  sdRef.initialize();
}

void PIDController::initializeUsingXML(TiXmlElement *element) {
  Signal::initializeUsingXML(element);
  sRef.initializeUsingXML(element);
  sdRef.initializeUsingXML(element);
  P.initializeUsingXML(element);
  I.initializeUsingXML(element);
  D.initializeUsingXML(element);
}

TiXmlElement* PIDController::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Signal::writeXMLFile(parent);
  sRef.writeXMLFile(ele0);
  sdRef.writeXMLFile(ele0);
  P.writeXMLFile(ele0);
  I.writeXMLFile(ele0);
  D.writeXMLFile(ele0);
  return ele0;
}

UnarySignalOperation::UnarySignalOperation(const string &str, Element *parent) : Signal(str, parent) {
  sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"inputSignal"));

  vector<Property*> property;
  property.push_back(new SymbolicFunction1Property("VV","x"));
  f.setProperty(new ChoiceProperty(MBSIMCONTROLNS"function",property));
}

void UnarySignalOperation::initialize() {
  Signal::initialize();
  sRef.initialize();
}

void UnarySignalOperation::initializeUsingXML(TiXmlElement *element) {
  Signal::initializeUsingXML(element);
  sRef.initializeUsingXML(element);
  f.initializeUsingXML(element);
}

TiXmlElement* UnarySignalOperation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Signal::writeXMLFile(parent);
  sRef.writeXMLFile(ele0);
  f.writeXMLFile(ele0);
  return ele0;
}

BinarySignalOperation::BinarySignalOperation(const string &str, Element *parent) : Signal(str, parent) {
  s1Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"input1Signal"));
  s2Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROLNS"input2Signal"));

  vector<Property*> property;
  vector<string> var;
  var.push_back("x1");
  var.push_back("x2");
  property.push_back(new SymbolicFunction2Property("VVV",var));
  f.setProperty(new ChoiceProperty(MBSIMCONTROLNS"function",property));
}

void BinarySignalOperation::initialize() {
  Signal::initialize();
  s1Ref.initialize();
  s2Ref.initialize();
}

void BinarySignalOperation::initializeUsingXML(TiXmlElement *element) {
  Signal::initializeUsingXML(element);
  s1Ref.initializeUsingXML(element);
  s2Ref.initializeUsingXML(element);
  f.initializeUsingXML(element);
}

TiXmlElement* BinarySignalOperation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Signal::writeXMLFile(parent);
  s1Ref.writeXMLFile(ele0);
  s2Ref.writeXMLFile(ele0);
  f.writeXMLFile(ele0);
  return ele0;
}
