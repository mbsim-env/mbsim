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
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Signal::Signal(const string &str) : Link(str) {
  }

  Signal::~Signal() {
  }

  PIDController::PIDController(const string &str) : Signal(str) {
    sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"inputSignal"));
    sdRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"derivativeOfInputSignal"));
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROL%"P"));
    P.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROL%"I"));
    I.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROL%"D"));
    D.setProperty(new ExtPhysicalVarProperty(input));
  }

  void PIDController::initialize() {
    Signal::initialize();
    sRef.initialize();
    sdRef.initialize();
  }

  DOMElement* PIDController::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    sRef.initializeUsingXML(element);
    sdRef.initializeUsingXML(element);
    P.initializeUsingXML(element);
    I.initializeUsingXML(element);
    D.initializeUsingXML(element);
    return element;
  }

  DOMElement* PIDController::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Signal::writeXMLFile(parent);
    sRef.writeXMLFile(ele0);
    sdRef.writeXMLFile(ele0);
    P.writeXMLFile(ele0);
    I.writeXMLFile(ele0);
    D.writeXMLFile(ele0);
    return ele0;
  }

  UnarySignalOperation::UnarySignalOperation(const string &str) : Signal(str) {
    sRef.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"inputSignal"));

    //  vector<Property*> property;
    //  vector<string> var;
    //  var.push_back("x");
    //  property.push_back(new SymbolicFunctionProperty("VV",var));
    //  f.setProperty(new ChoiceProperty(MBSIMCONTROL%"function",property));
    f.setProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory3(this,"VV",vector<string>(1,"x")),MBSIMCONTROL%"function"));
  }

  void UnarySignalOperation::initialize() {
    Signal::initialize();
    sRef.initialize();
  }

  DOMElement* UnarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    sRef.initializeUsingXML(element);
    f.initializeUsingXML(element);
    return element;
  }

  DOMElement* UnarySignalOperation::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Signal::writeXMLFile(parent);
    sRef.writeXMLFile(ele0);
    f.writeXMLFile(ele0);
    return ele0;
  }

  BinarySignalOperation::BinarySignalOperation(const string &str) : Signal(str) {
    s1Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"firstInputSignal"));
    s2Ref.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"secondInputSignal"));

    vector<string> var;
    var.push_back("x1");
    var.push_back("x2");
    f.setProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2(this,"VVV",var),MBSIMCONTROL%"function"));
  }

  void BinarySignalOperation::initialize() {
    Signal::initialize();
    s1Ref.initialize();
    s2Ref.initialize();
  }

  DOMElement* BinarySignalOperation::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    s1Ref.initializeUsingXML(element);
    s2Ref.initializeUsingXML(element);
    f.initializeUsingXML(element);
    return element;
  }

  DOMElement* BinarySignalOperation::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Signal::writeXMLFile(parent);
    s1Ref.writeXMLFile(ele0);
    s2Ref.writeXMLFile(ele0);
    f.writeXMLFile(ele0);
    return ele0;
  }

  ExternSignalSource::ExternSignalSource(const string &str) : Signal(str) {
    sourceSize.setProperty(new IntegerProperty(1,MBSIMCONTROL%"sourceSize"));
  }

  void ExternSignalSource::initialize() {
    Signal::initialize();
    sourceSize.initialize();
  }

  DOMElement* ExternSignalSource::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    sourceSize.initializeUsingXML(element);
    return element;
  }

  DOMElement* ExternSignalSource::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Signal::writeXMLFile(parent);
    sourceSize.writeXMLFile(ele0);
    return ele0;
  }

  ExternSignalSink::ExternSignalSink(const string &str) : Signal(str) {
    inputSignal.setProperty(new SignalOfReferenceProperty("",this,MBSIMCONTROL%"inputSignal"));
  }

  void ExternSignalSink::initialize() {
    Signal::initialize();
    inputSignal.initialize();
  }

  DOMElement* ExternSignalSink::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    inputSignal.initializeUsingXML(element);
    return element;
  }

  DOMElement* ExternSignalSink::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Signal::writeXMLFile(parent);
    inputSignal.writeXMLFile(ele0);
    return ele0;
  }

}
