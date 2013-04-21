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
#include "embedded_elements.h"
#include "basic_properties.h"

using namespace std;

EmbeddedObject::EmbeddedObject(const string &str, Element *parent) : Object(str,parent), count(0,false), counterName(0,false), parameterList(0,false) {
  href.setProperty(new FileProperty(""));
//  vector<PhysicalStringProperty*> input;
//  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",""));
//  count.setProperty(new ExtPhysicalVarProperty(input));
  count.setProperty(new TextProperty("",""));
  counterName.setProperty(new TextProperty("",""));
  parameterList.setProperty(new FileProperty(""));
}

EmbeddedObject::~EmbeddedObject() {
}

void EmbeddedObject::initializeUsingXML(TiXmlElement *element) {
  string file = element->Attribute("href");
  static_cast<FileProperty*>(href.getProperty())->setFileName(file);
  static_cast<FileProperty*>(href.getProperty())->setAbsoluteFilePath(file);
  if(element->Attribute("count")) {
    count.setActive(true);
    static_cast<TextProperty*>(count.getProperty())->setText(element->Attribute("count"));
  }
  if(element->Attribute("counterName")) {
    counterName.setActive(true);
    static_cast<TextProperty*>(counterName.getProperty())->setText(element->Attribute("counterName"));
  }
  TiXmlElement *ele = element->FirstChildElement(PVNS+string("localParameter"));
  if(ele) {
    parameterList.setActive(true);
    string file = ele->Attribute("href");
    static_cast<FileProperty*>(parameterList.getProperty())->setFileName(file);
    static_cast<FileProperty*>(parameterList.getProperty())->setAbsoluteFilePath(file);
  }
}

TiXmlElement* EmbeddedObject::writeXMLFile(TiXmlNode *parent) {    
  TiXmlElement *ele0=new TiXmlElement(PVNS+string("embed"));
  ele0->SetAttribute("href", static_cast<FileProperty*>(href.getProperty())->getAbsoluteFilePath());
  if(count.isActive())
    ele0->SetAttribute("count", static_cast<TextProperty*>(count.getProperty())->getText());
  //if(static_cast<TextProperty*>(counterName.getProperty())->getText() != "")
  if(counterName.isActive())
    ele0->SetAttribute("counterName", static_cast<TextProperty*>(counterName.getProperty())->getText());
  if(parameterList.isActive()) {
    TiXmlElement *ele1=new TiXmlElement(PVNS+string("localParameter"));
    ele1->SetAttribute("href", static_cast<FileProperty*>(parameterList.getProperty())->getAbsoluteFilePath());
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}
