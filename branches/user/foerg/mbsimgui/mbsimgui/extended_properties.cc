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

ExtPhysicalVarProperty::ExtPhysicalVarProperty(const std::vector<PhysicalVariableProperty> &inputProperty_) : inputProperty(inputProperty_), currentInput(0) {
  inputProperty.push_back(PhysicalVariableProperty(new OctaveExpressionProperty, inputProperty[0].getUnit(), inputProperty[0].getXmlName()));
}

TiXmlElement* ExtPhysicalVarProperty::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i< inputProperty.size(); i++) {
    if(inputProperty[i].initializeUsingXML(element)) { 
      currentInput = i;
      return element;
    }
  }
  return 0;
}

TiXmlElement* ExtPhysicalVarProperty::writeXMLFile(TiXmlNode *parent) {
  inputProperty[currentInput].writeXMLFile(parent);
  return 0;
}

void ExtPhysicalVarProperty::fromWidget(QWidget *widget) {
  currentInput = static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentInput();
  inputProperty[currentInput].fromWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentPhysicalVariableWidget());
}

void ExtPhysicalVarProperty::toWidget(QWidget *widget) {
  static_cast<ExtPhysicalVarWidget*>(widget)->setCurrentInput(currentInput);
  inputProperty[currentInput].toWidget(static_cast<ExtPhysicalVarWidget*>(widget)->getCurrentPhysicalVariableWidget());
}

ChoiceProperty2::ChoiceProperty2(PropertyFactory *factory_, const std::string &xmlName_, int mode_, const std::string &xmlBase_) : factory(factory_), index(0), mode(mode_), xmlName(xmlName_), xmlBase(xmlBase_), property(factory->createProperty()) {
}

ChoiceProperty2::ChoiceProperty2(const ChoiceProperty2 &p) : index(p.index), mode(p.mode), xmlName(p.xmlName), xmlBase(p.xmlBase) {
//  for(unsigned int i=0; i<p.property.size(); i++)
//    property.push_back(p.property[i]->clone());
}

ChoiceProperty2::~ChoiceProperty2() {
//  for(unsigned int i=0; i<property.size(); i++)
//    delete property[i];
}

ChoiceProperty2& ChoiceProperty2::operator=(const ChoiceProperty2 &p) {
//  for(unsigned int i=0; i<property.size(); i++)
//    delete property[i];
//  property.clear();
//  index=p.index; 
//  mode=p.mode; 
//  xmlName=p.xmlName; 
//  xmlBase=p.xmlBase;
//  for(unsigned int i=0; i<p.property.size(); i++)
//    property.push_back(p.property[i]->clone());
}

void ChoiceProperty2::initialize() {
  property->initialize();
}

TiXmlElement* ChoiceProperty2::initializeUsingXML(TiXmlElement *element) {
  if(element) {
    if(mode<=1) {
      TiXmlElement *e=(xmlName!="")?element->FirstChildElement(xmlName):element;
      if(e) {
        TiXmlElement* ee=(mode==0)?e->FirstChildElement():e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            if(ee->ValueStr() == factory->getName(i)) {
              index = i;
              property = factory->createProperty(i);
              return property->initializeUsingXML(ee);
            }
          }
        }
      }
      return 0;
    }
    else if (mode<=3) {
      TiXmlElement *e=(xmlName!="")?element->FirstChildElement(xmlName):element;
      if(e) {
        TiXmlElement* ee=(mode==2)?e->FirstChildElement():e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            TiXmlElement *eee=ee->FirstChildElement(factory->getName(i));
            if(eee) {
              index = i;
              property = factory->createProperty(i);
              return property->initializeUsingXML(ee);
            }
          }
        }
      }
      return 0;
    }
    else {
      TiXmlElement *e=(xmlName!="")?element->FirstChildElement(xmlName):element;
      if(e) {
        TiXmlElement* ee=e;
        if(ee) {
          for(int i=0; i<factory->getSize(); i++) {
            TiXmlElement *eee=ee->FirstChildElement();
            if(eee) {
              index = i;
              property = factory->createProperty(i);
              if(property->initializeUsingXML(ee))
                return eee;
            }
          }
        }
      }
      return 0;
    }
  }
}

TiXmlElement* ChoiceProperty2::writeXMLFile(TiXmlNode *parent) {
  TiXmlNode *ele0;
  if(xmlName!="") {
    ele0 = new TiXmlElement(xmlName);
    parent->LinkEndChild(ele0);
  }
  else
    ele0 = parent;
  property->writeXMLFile(ele0);

  return 0;
}

void ChoiceProperty2::fromWidget(QWidget *widget) {
  int newindex = static_cast<ChoiceWidget2*>(widget)->comboBox->currentIndex();
  if(index != newindex) {
    index = newindex;
    delete property;
    property = factory->createProperty(index);
  }
  property->fromWidget(static_cast<ChoiceWidget2*>(widget)->getWidget());
}

void ChoiceProperty2::toWidget(QWidget *widget) {
  static_cast<ChoiceWidget2*>(widget)->comboBox->blockSignals(true);
  static_cast<ChoiceWidget2*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<ChoiceWidget2*>(widget)->comboBox->blockSignals(false);
  static_cast<ChoiceWidget2*>(widget)->blockSignals(true);
  static_cast<ChoiceWidget2*>(widget)->defineWidget(index);
  static_cast<ChoiceWidget2*>(widget)->blockSignals(false);
  property->toWidget(static_cast<ChoiceWidget2*>(widget)->getWidget());
}

TiXmlElement* ExtProperty::initializeUsingXML(TiXmlElement *element) {
  active = false;
  if(element)
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e)
      active = property->initializeUsingXML(e);
    if(alwaysWriteXMLName) 
      return e;
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
  if(active)
    property->fromWidget(static_cast<ExtWidget*>(widget)->widget);
}

void ExtProperty::toWidget(QWidget *widget) {
  static_cast<ExtWidget*>(widget)->blockSignals(true);
  static_cast<ExtWidget*>(widget)->setActive(active);
  static_cast<ExtWidget*>(widget)->setWidgetVisible(active);
  static_cast<ExtWidget*>(widget)->blockSignals(false);
  if(active)
    property->toWidget(static_cast<ExtWidget*>(widget)->widget);
}

ContainerProperty::ContainerProperty(const ContainerProperty &p) : xmlName(p.xmlName) {
  for(unsigned int i=0; i<p.property.size(); i++)
    property.push_back(p.property[i]->clone());
}

ContainerProperty::~ContainerProperty() {
  for(vector<Property*>::iterator i = property.begin(); i != property.end(); ++i)
    delete *i;
}

ContainerProperty& ContainerProperty::operator=(const ContainerProperty &p) {
  for(vector<Property*>::iterator i = property.begin(); i != property.end(); ++i)
    delete *i;
  property.clear();
  xmlName=p.xmlName; 
  for(unsigned int i=0; i<p.property.size(); i++)
    property.push_back(p.property[i]->clone());
}

void ContainerProperty::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

TiXmlElement* ContainerProperty::initializeUsingXML(TiXmlElement *element) {
  bool flag = false;
  if(mode==0) {
  if(xmlName!="") {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    if(e) {
      for(unsigned int i=0; i<property.size(); i++)
        if(property[i]->initializeUsingXML(e))
          flag = true;
    }
    return flag?e:0;
  }
  else {
    for(unsigned int i=0; i<property.size(); i++)
      if(property[i]->initializeUsingXML(element))
        flag = true;
    return flag?element:0;
  }
  }
  else {
  if(xmlName!="") {
    TiXmlElement *e=element;
    if(e) {
      for(unsigned int i=0; i<property.size(); i++) {
        if(property[i]->initializeUsingXML(e))
          flag = true;
      }
    }
    return flag?e:0;
  }
  else {
    for(unsigned int i=0; i<property.size(); i++)
      if(property[i]->initializeUsingXML(element))
        flag = true;
    return flag?element:0;
  }
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

ListProperty::ListProperty(PropertyFactory *factory_, const string &xmlName_, int m, int mode_) : factory(factory_), xmlName(xmlName_), mode(mode_) {
  for(int i=0; i<m; i++)
    property.push_back(factory->createProperty());
}

TiXmlElement* ListProperty::initializeUsingXML(TiXmlElement *element) {
  
  property.clear();
  if(xmlName=="") {
    TiXmlElement *e=(mode==0)?element->FirstChildElement():element;
    while(e) {
      property.push_back(factory->createProperty());
      property[property.size()-1]->initializeUsingXML(e);

      e=e->NextSiblingElement();
    }
  }
  else {
    TiXmlElement *e=element->FirstChildElement(xmlName);
    while(e and e->ValueStr()==xmlName) {
      property.push_back(factory->createProperty());
      property[property.size()-1]->initializeUsingXML(e);

      e=e->NextSiblingElement();
    }
  }

  return element;
}

TiXmlElement* ListProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName=="") {
    for(unsigned int i=0; i<property.size(); i++)
      property[i]->writeXMLFile(parent);
  }
  else {
    for(unsigned int i=0; i<property.size(); i++) {
      TiXmlElement *ele0 = new TiXmlElement(xmlName);
      property[i]->writeXMLFile(ele0);
      parent->LinkEndChild(ele0);
    }
  }
  return 0;
}

void ListProperty::fromWidget(QWidget *widget) {
  if(property.size()!=static_cast<ListWidget*>(widget)->stackedWidget->count()) {
    property.clear();
    for(unsigned int i=0; i<static_cast<ListWidget*>(widget)->stackedWidget->count(); i++) {
      property.push_back(factory->createProperty());
      property[i]->fromWidget(static_cast<ListWidget*>(widget)->stackedWidget->widget(i));
    }
  }
  else {
    for(unsigned int i=0; i<static_cast<ListWidget*>(widget)->stackedWidget->count(); i++) {
      property[i]->fromWidget(static_cast<ListWidget*>(widget)->stackedWidget->widget(i));
    }
  }
}

void ListProperty::toWidget(QWidget *widget) {
  static_cast<ListWidget*>(widget)->blockSignals(true);
  if(property.size()!=static_cast<ListWidget*>(widget)->stackedWidget->count()) {
    static_cast<ListWidget*>(widget)->removeElements(static_cast<ListWidget*>(widget)->stackedWidget->count());

    static_cast<ListWidget*>(widget)->spinBox->blockSignals(true);
    static_cast<ListWidget*>(widget)->spinBox->setValue(property.size());
    static_cast<ListWidget*>(widget)->spinBox->blockSignals(false);

    static_cast<ListWidget*>(widget)->addElements(property.size(),false);
    for(unsigned int i=0; i<property.size(); i++)
      property[i]->toWidget(static_cast<ListWidget*>(widget)->stackedWidget->widget(i));
  }
  else {
    for(unsigned int i=0; i<property.size(); i++)
      property[i]->toWidget(static_cast<ListWidget*>(widget)->stackedWidget->widget(i));
  }
  static_cast<ListWidget*>(widget)->blockSignals(false);
}

void ListProperty::initialize() {
  for(unsigned int i=0; i<property.size(); i++)
    property[i]->initialize();
}

Property* ChoicePropertyFactory::createProperty(int i) {
  return new ChoiceProperty2(factory,xmlName,mode);
}


