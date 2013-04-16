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
#include "objectfactory.h"

using namespace std;

Parameter::Parameter(const string &name_) : name(name_) {
}

Parameter::~Parameter() {
}

void Parameter::initializeUsingXML(TiXmlElement *element) {
}

TiXmlElement* Parameter::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PARAMNS+getType());
  ele0->SetAttribute("name", getName());
  TiXmlText* text= new TiXmlText(getValue());
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);
  return ele0;
}

Parameter* Parameter::readXMLFile(const string &filename) {
  MBSimObjectFactory::initialize();
  TiXmlDocument doc;
  bool ret=doc.LoadFile(filename);
  assert(ret==true);
  TiXml_PostLoadFile(&doc);
  TiXmlElement *e=doc.FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(e, dummy);
  Parameter *parameter=ObjectFactory::getInstance()->createParameter(e);
  parameter->initializeUsingXML(doc.FirstChildElement());
  return parameter;
}

void Parameter::writeXMLFile(const string &name) {
  TiXmlDocument doc;
  TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","UTF-8","");
  doc.LinkEndChild( decl );
  writeXMLFile(&doc);
  unIncorporateNamespace(doc.FirstChildElement(), Utils::getMBSimNamespacePrefixMapping());  
  doc.SaveFile((name.length()>15 && name.substr(name.length()-15,15)==".mbsimparam.xml")?name:name+".mbsimparam.xml");
}

ScalarParameter::ScalarParameter(const string &str) : Parameter(str) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));
}

string ScalarParameter::getValue() const { 
  return static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue();
}

void ScalarParameter::initializeUsingXML(TiXmlElement *element) {
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
}

VectorParameter::VectorParameter(const string &str) : Parameter(str) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new VecProperty(3),"",""));
  value.setProperty(new ExtPhysicalVarProperty(input));
}

string VectorParameter::getValue() const { 
  return static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue();
}

void VectorParameter::initializeUsingXML(TiXmlElement *element) {
  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  val->initializeUsingXML(element);
}

