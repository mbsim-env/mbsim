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
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include <QStackedWidget>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;

ExtPhysicalVarProperty::ExtPhysicalVarProperty(std::vector<PhysicalStringProperty*> inputProperty_) : inputProperty(inputProperty_), currentInput(0) {
  inputProperty.push_back(new PhysicalStringProperty(new OctaveExpressionProperty, inputProperty[0]->getUnit(), inputProperty[0]->getXmlName()));
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
  for(vector<PhysicalStringProperty*>::iterator i = inputProperty.begin(); i != inputProperty.end(); ++i)
    delete *i;
}

TiXmlElement* ExtPhysicalVarProperty::writeXMLFile(TiXmlNode *parent) {
  inputProperty[currentInput]->writeXMLFile(parent);
  return 0;
}

void ExtPhysicalVarProperty::fromWidget(QWidget *widget) {
  currentInput = static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentInput();
  for(int i=0; i< inputProperty.size(); i++)
    inputProperty[i]->fromWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getPhysicalStringWidget(i));
}

void ExtPhysicalVarProperty::toWidget(QWidget *widget) {
  static_cast<ExtPhysicalVarWidget*>(widget)->setCurrentInput(currentInput);
  for(int i=0; i< inputProperty.size(); i++)
    inputProperty[i]->toWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getPhysicalStringWidget(i));
}

PropertyChoiceProperty::~PropertyChoiceProperty() {
  for(vector<Property*>::iterator i = property.begin(); i != property.end(); ++i)
    delete *i;
}

void PropertyChoiceProperty::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

TiXmlElement* PropertyChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i<property.size(); i++)
    if(property[i]->initializeUsingXML(element)) {
      index = i;
      return element;
    }
  return 0;
}

TiXmlElement* PropertyChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  return property[index]->writeXMLFile(parent);
}

void PropertyChoiceProperty::fromWidget(QWidget *widget) {
  index = static_cast<WidgetChoiceWidget*>(widget)->stackedWidget->currentIndex();
  property[index]->fromWidget(static_cast<WidgetChoiceWidget*>(widget)->stackedWidget->currentWidget());
}

void PropertyChoiceProperty::toWidget(QWidget *widget) {
  static_cast<WidgetChoiceWidget*>(widget)->changeCurrent(index);
  property[index]->toWidget(static_cast<WidgetChoiceWidget*>(widget)->stackedWidget->currentWidget());
}

TiXmlElement* ExtProperty::initializeUsingXML(TiXmlElement *element) {
  active = false;
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e)
      active = property->initializeUsingXML(e);
  }
  else {
    active = property->initializeUsingXML(element);
  }
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

PropertyContainer::~PropertyContainer() {
  for(vector<Property*>::iterator i = property.begin(); i != property.end(); ++i)
    delete *i;
}

void PropertyContainer::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

TiXmlElement* PropertyContainer::initializeUsingXML(TiXmlElement *element) {
  for(unsigned int i=0; i<property.size(); i++)
    if(property[i]->initializeUsingXML(element))
      return 0;
  return element;
}

TiXmlElement* PropertyContainer::writeXMLFile(TiXmlNode *parent) {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->writeXMLFile(parent);
  return 0;
}

void PropertyContainer::fromWidget(QWidget *widget) {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->fromWidget(static_cast<WidgetContainer*>(widget)->widget[i]);
}

void PropertyContainer::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->toWidget(static_cast<WidgetContainer*>(widget)->widget[i]);
}


