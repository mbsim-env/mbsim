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

#ifndef _EXTENDED_PROPERTIES_H_
#define _EXTENDED_PROPERTIES_H_

#include "variable_properties.h"

class ExtPhysicalVarProperty : public Property {

  public:
    ExtPhysicalVarProperty(std::vector<PhysicalVariableProperty*> inputProperty);
    ~ExtPhysicalVarProperty();
    PhysicalVariableProperty* getPhysicalVariableProperty(int i) {return inputProperty[i];}
    PhysicalVariableProperty* getCurrentPhysicalVariableProperty() {return inputProperty[currentInput];}
    const PhysicalVariableProperty* getCurrentPhysicalVariableProperty() const {return inputProperty[currentInput];}
    int getNumberOfInputs() const {return inputProperty.size();}
    std::string getValue() const {return inputProperty[currentInput]->getValue();}
    void setValue(const std::string &str) {inputProperty[currentInput]->setValue(str);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<PhysicalVariableProperty*> inputProperty;
    int currentInput;
};

class ExtProperty : public Property {
  public:
    ExtProperty(Property *property_=0, bool active_=true, const std::string &xmlName_="", bool flag=true) : property(property_), active(active_), xmlName(xmlName_), alwaysWriteXMLName(flag) {}
    ~ExtProperty() {delete property;}
    Property* getProperty() {return property;}
    const Property* getProperty() const {return property;}
    void setProperty(Property *property_) {property = property_;}
    void setXMLName(const std::string &xmlName_, bool flag=true) {xmlName = xmlName_; alwaysWriteXMLName=flag;}

    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
    void initialize() {property->initialize();}
    bool isActive() const {return active;}
    void setActive(bool active_) {active = active_;}

  protected:
    Property *property;
    std::string xmlName;
    bool active, alwaysWriteXMLName;
};

class ChoiceProperty : public Property {

  public:
    ChoiceProperty(const std::string &xmlName_, const std::vector<Property*> &property_, int mode_=0, const std::string &xmlBase_=MBSIMNS) : property(property_), index(0), mode(mode_), xmlName(xmlName_), xmlBase(xmlBase_) {}
    ~ChoiceProperty();

    void initialize();
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<Property*> property;
    int index, mode;
    std::string xmlName, xmlBase;
};

class ContainerProperty : public Property {
  public:
    ContainerProperty(const std::string &xmlName_="") : xmlName(xmlName_) {}
    ContainerProperty(const std::vector<Property*> &property_, const std::string &xmlName_="") : property(property_), xmlName(xmlName_) {}
    ~ContainerProperty();

    void initialize();
    void addProperty(Property *property_) {property.push_back(property_);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    std::vector<Property*> property;
    std::string xmlName;
};

#endif

