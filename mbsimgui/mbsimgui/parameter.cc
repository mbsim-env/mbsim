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
#include "parameter.h"
#include "basic_properties.h"
#include "objectfactory.h"

using namespace std;
using namespace MBXMLUtils;

Parameter::Parameter(const string &name_) {
  name.setProperty(new TextProperty(name_,""));
}

Parameter::~Parameter() {
}

  //return static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue();

void Parameter::initializeUsingXML(TiXmlElement *element) {
}

TiXmlElement* Parameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+getType());
  ele0->SetAttribute("name", getName());
  parent->LinkEndChild(ele0);
  return ele0;
}

ScalarParameter::ScalarParameter(const string &name) : Parameter(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));
  setValue(static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue());
}

ScalarParameter::~ScalarParameter() {
}

void ScalarParameter::initializeUsingXML(TiXmlElement *element) {
  Parameter::initializeUsingXML(element);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
  setValue(val->getValue());
}

TiXmlElement* ScalarParameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Parameter::writeXMLFile(parent);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->writeXMLFile(ele0);
  return ele0;
}

VectorParameter::VectorParameter(const string &name) : Parameter(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));
  setValue(static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue());
}

VectorParameter::~VectorParameter() {
}

void VectorParameter::initializeUsingXML(TiXmlElement *element) {
  Parameter::initializeUsingXML(element);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
  setValue(val->getValue());
}

TiXmlElement* VectorParameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Parameter::writeXMLFile(parent);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->writeXMLFile(ele0);
  return ele0;
}

MatrixParameter::MatrixParameter(const string &name) : Parameter(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new MatProperty(3,3),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));
  setValue(static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue());
}

MatrixParameter::~MatrixParameter() {
}

void MatrixParameter::initializeUsingXML(TiXmlElement *element) {
  Parameter::initializeUsingXML(element);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
  setValue(val->getValue());
}

TiXmlElement* MatrixParameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Parameter::writeXMLFile(parent);
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->writeXMLFile(ele0);
  return ele0;
}

void ParameterList::addParameterList(const ParameterList &list) {
  for(int i=0; i<list.getSize(); i++)
    addParameter(list.getParameterName(i),list.getParameterValue(i));
}

bool ParameterList::readXMLFile(const string &filename) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    TiXmlElement *E=e->FirstChildElement();
    vector<QString> refFrame;
    while(E) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(E);
      parameter->initializeUsingXML(E);
      addParameter(parameter->getName(),parameter->getValue());
      delete parameter;
      E=E->NextSiblingElement();
    }
    return true;
  }
  return false;
}
