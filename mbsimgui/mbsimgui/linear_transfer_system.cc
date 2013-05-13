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
#include "linear_transfer_system.h"

using namespace std;
using namespace MBXMLUtils;

LinearTransferSystem::LinearTransferSystem(const string &str, Element *parent) : SignalProcessingSystem(str, parent) {
  PropertyContainer *propertyContainer = new PropertyContainer(MBSIMCONTROLNS"pidType");
  vector<Property*> choiceProperty;

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROLNS"P"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"I"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"D"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  propertyContainer = new PropertyContainer(MBSIMCONTROLNS"abcdType");

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"A"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"B"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"C"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMCONTROLNS"D"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  propertyContainer = new PropertyContainer(MBSIMCONTROLNS"integratorType");

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROLNS"gain"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  propertyContainer = new PropertyContainer(MBSIMCONTROLNS"pt1Type");

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMCONTROLNS"P"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.1"),"-",MBSIMCONTROLNS"T"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  choice.setProperty(new PropertyChoiceProperty(choiceProperty));
}

void LinearTransferSystem::initializeUsingXML(TiXmlElement *element) {
  SignalProcessingSystem::initializeUsingXML(element);
  choice.initializeUsingXML(element);
}

TiXmlElement* LinearTransferSystem::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = SignalProcessingSystem::writeXMLFile(parent);
  cout << choice.isActive() << endl;
  choice.writeXMLFile(ele0);
  return ele0;
}
