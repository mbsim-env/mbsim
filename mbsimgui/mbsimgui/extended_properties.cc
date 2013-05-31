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
#include "extended_properties.h"
#include "frame.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include <QStackedWidget>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;
using namespace MBXMLUtils;

ExtPhysicalVarProperty::ExtPhysicalVarProperty(std::vector<PhysicalVariableProperty*> inputProperty_) : inputProperty(inputProperty_), currentInput(0) {
  inputProperty.push_back(new PhysicalVariableProperty(new OctaveExpressionProperty, inputProperty[0]->getUnit(), inputProperty[0]->getXmlName()));
}

TiXmlElement* ExtPhysicalVarProperty::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i< inputProperty.size(); i++) {
    if(inputProperty[i]->initializeUsingXML(element)) { 
      currentInput = i;
      return element;
    }
  }
  return 0;
}

ExtPhysicalVarProperty::~ExtPhysicalVarProperty() {
  for(vector<PhysicalVariableProperty*>::iterator i = inputProperty.begin(); i != inputProperty.end(); ++i)
    delete *i;
}

TiXmlElement* ExtPhysicalVarProperty::writeXMLFile(TiXmlNode *parent) {
  inputProperty[currentInput]->writeXMLFile(parent);
  return 0;
}

void ExtPhysicalVarProperty::fromWidget(QWidget *widget) {
  currentInput = static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentInput();
  inputProperty[currentInput]->fromWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentPhysicalVariableWidget());
//  for(int i=0; i< inputProperty.size(); i++)
//    inputProperty[i]->fromWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getPhysicalVariableWidget(i));
}

void ExtPhysicalVarProperty::toWidget(QWidget *widget) {
  static_cast<ExtPhysicalVarWidget*>(widget)->setCurrentInput(currentInput);
  inputProperty[currentInput]->toWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentPhysicalVariableWidget());
  //for(int i=0; i< inputProperty.size(); i++)
  //  inputProperty[i]->toWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getPhysicalVariableWidget(i));
}

ChoiceProperty::~ChoiceProperty() {
  for(unsigned int i=0; i<property.size(); i++)
    delete property[i];
}

void ChoiceProperty::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

TiXmlElement* ChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  if(mode==0) {
    TiXmlElement *e=(xmlName!="")?element->FirstChildElement(xmlName):element;
    if(e) {
      TiXmlElement* ee=e->FirstChildElement();
      if(ee) {
        for(int i=0; i<property.size(); i++)
          if(ee->ValueStr() == MBSIMNS+property[i]->getType()) {
            index = i;
            break;
          }
        property[index]->initializeUsingXML(ee);
      }
    }
    return e;
  }
  else {
    if(xmlName!="") {
      TiXmlElement *e=element->FirstChildElement(xmlName);
      if(e)
        for(int i=0; i<property.size(); i++)
          if(property[i]->initializeUsingXML(e)) {
            index = i;
            return e;
          }
    }
    else {
      for(int i=0; i<property.size(); i++)
        if(property[i]->initializeUsingXML(element)) {
          index = i;
          return element;
        }
    }
    return 0;
  }
}

TiXmlElement* ChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlNode *ele0;
  if(xmlName!="") {
    ele0 = new TiXmlElement(xmlName);
    parent->LinkEndChild(ele0);
  }
  else
    ele0 = parent;
  property[index]->writeXMLFile(ele0);

  return 0;
}

void ChoiceProperty::fromWidget(QWidget *widget) {
  index = static_cast<ChoiceWidget*>(widget)->comboBox->currentIndex();
  property[index]->fromWidget(static_cast<ChoiceWidget*>(widget)->getWidget());
}

void ChoiceProperty::toWidget(QWidget *widget) {
  static_cast<ChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<ChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<ChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<ChoiceWidget*>(widget)->blockSignals(true);
  static_cast<ChoiceWidget*>(widget)->defineWidget(index);
  static_cast<ChoiceWidget*>(widget)->blockSignals(false);
  property[index]->toWidget(static_cast<ChoiceWidget*>(widget)->getWidget());
}

TiXmlElement* ExtProperty::initializeUsingXML(TiXmlElement *element) {
  active = false;
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e)
      active = property->initializeUsingXML(e);
  }
  else
    active = property->initializeUsingXML(element);
  return active?element:0;
}

TiXmlElement* ExtProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    if(alwaysWriteXMLName) {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      if(active) property->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
      return ele0;
    }
    else if(active) {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      property->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
      return ele0;
    }
  }
  else
    return active?property->writeXMLFile(parent):0;
}

void ExtProperty::fromWidget(QWidget *widget) {
  active = static_cast<ExtWidget*>(widget)->isActive();
  property->fromWidget(static_cast<ExtWidget*>(widget)->widget);
}

void ExtProperty::toWidget(QWidget *widget) {
  static_cast<ExtWidget*>(widget)->blockSignals(true);
  static_cast<ExtWidget*>(widget)->setActive(active);
  static_cast<ExtWidget*>(widget)->setWidgetVisible(active);
  static_cast<ExtWidget*>(widget)->blockSignals(false);
  property->toWidget(static_cast<ExtWidget*>(widget)->widget);
}

ContainerProperty::~ContainerProperty() {
  for(vector<Property*>::iterator i = property.begin(); i != property.end(); ++i)
    delete *i;
}

void ContainerProperty::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

TiXmlElement* ContainerProperty::initializeUsingXML(TiXmlElement *element) {
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e)
      for(unsigned int i=0; i<property.size(); i++)
        if(!property[i]->initializeUsingXML(e))
          return 0;
    return e;
  }
  else {
    for(unsigned int i=0; i<property.size(); i++)
      if(!property[i]->initializeUsingXML(element))
        return 0;
    return element;
  }
}

TiXmlElement* ContainerProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    for(unsigned int i=0; i<property.size(); i++)
      property[i]->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    for(unsigned int i=0; i<property.size(); i++)
      property[i]->writeXMLFile(parent);
  return 0;
}

void ContainerProperty::fromWidget(QWidget *widget) {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->fromWidget(static_cast<ContainerWidget*>(widget)->widget[i]);
}

void ContainerProperty::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->toWidget(static_cast<ContainerWidget*>(widget)->widget[i]);
}


