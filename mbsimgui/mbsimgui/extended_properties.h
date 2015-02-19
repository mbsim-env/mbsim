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

namespace MBSimGUI {

  class ExtPhysicalVarProperty : public Property {

    public:
      ExtPhysicalVarProperty(const std::vector<PhysicalVariableProperty> &inputProperty);
      virtual PropertyInterface* clone() const {return new ExtPhysicalVarProperty(*this);}
      const PhysicalVariableProperty& getPhysicalVariableProperty(int i) {return inputProperty[i];}
      const PhysicalVariableProperty& getCurrentPhysicalVariableProperty() {return inputProperty[currentInput];}
      const PhysicalVariableProperty& getCurrentPhysicalVariableProperty() const {return inputProperty[currentInput];}
      int getNumberOfInputs() const {return inputProperty.size();}
      std::string getValue() const {return inputProperty[currentInput].getValue();}
      void setValue(const std::string &str) {inputProperty[currentInput].setValue(str);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      std::vector<PhysicalVariableProperty> inputProperty;
      int currentInput;
  };

  class ExtProperty : public Property {
    public:
      ExtProperty(PropertyInterface *property_=0, bool active_=true, const MBXMLUtils::FQN &xmlName_="", bool alwaysWriteXMLName_=true) : property(property_), xmlName(xmlName_), active(active_), alwaysWriteXMLName(alwaysWriteXMLName_) {}
      ExtProperty(const ExtProperty &p) : property(p.property?p.property->clone():0), xmlName(p.xmlName), active(p.active), alwaysWriteXMLName(p.alwaysWriteXMLName) {}
      ~ExtProperty() {delete property;}
      ExtProperty& operator=(const ExtProperty &p) {delete property; property=p.property?p.property->clone():0; xmlName=p.xmlName; active=p.active; alwaysWriteXMLName=p.alwaysWriteXMLName; return *this; }
      virtual PropertyInterface* clone() const {return new ExtProperty(*this);}
      PropertyInterface* getProperty() {return property;}
      const PropertyInterface* getProperty() const {return property;}
      void setProperty(PropertyInterface *property_) {property = property_;}
      void setXMLName(const MBXMLUtils::FQN &xmlName_, bool alwaysWriteXMLName_=true) {xmlName = xmlName_; alwaysWriteXMLName=alwaysWriteXMLName_;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void initialize() {property->initialize();}
      bool isActive() const {return active;}
      void setActive(bool active_) {active = active_;}

    protected:
      PropertyInterface *property;
      MBXMLUtils::FQN xmlName;
      bool active, alwaysWriteXMLName;
  };

  class ChoiceProperty2 : public Property {

    public:
      ChoiceProperty2(PropertyFactory *factory_, const MBXMLUtils::FQN &xmlName_, int mode_=0, const MBXMLUtils::NamespaceURI &xmlBase_=MBSIM); 
      ChoiceProperty2(const ChoiceProperty2 &p);
      ~ChoiceProperty2();
      ChoiceProperty2& operator=(const ChoiceProperty2 &p);
      virtual PropertyInterface* clone() const {return new ChoiceProperty2(*this);}

      void initialize();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      PropertyInterface* getProperty() const {return property;}
      void setProperty(PropertyInterface *property_) {property = property_;}

    protected:
      PropertyFactory *factory;
      PropertyInterface *property;
      int index, mode;
      MBXMLUtils::FQN xmlName;
      MBXMLUtils::NamespaceURI nsuri;
  };

  class ContainerProperty : public Property {
    public:
      ContainerProperty(const MBXMLUtils::FQN &xmlName_="", int mode_=1) : xmlName(xmlName_), mode(mode_) {}
      ContainerProperty(const std::vector<PropertyInterface*> &property_, const MBXMLUtils::FQN &xmlName_="") : property(property_), xmlName(xmlName_) {}
      ContainerProperty(const ContainerProperty &p);
      ~ContainerProperty();
      ContainerProperty& operator=(const ContainerProperty &p);
      virtual PropertyInterface* clone() const {return new ContainerProperty(*this);}

      void initialize();
      void addProperty(PropertyInterface *property_) {property.push_back(property_);}
      PropertyInterface* getProperty(int i) const { return property[i]; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      std::vector<PropertyInterface*> property;
      MBXMLUtils::FQN xmlName;
      int mode;
  };

  class ListProperty : public Property {
    public:
      ListProperty(PropertyFactory *factory, const MBXMLUtils::FQN &xmlName="", int m=0, int mode=0);
      ~ListProperty() { delete factory; }
      virtual PropertyInterface* clone() const {return new ListProperty(*this);}
      int getSize() const { return property.size(); }
      PropertyInterface* getProperty(int i) const { return property[i]; }
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
      void initialize();
    protected:
      std::vector<PropertyInterface*> property;
      PropertyFactory *factory;
      MBXMLUtils::FQN xmlName;
      int mode;
  };

  class ChoicePropertyFactory : public PropertyFactory {
    public:
      ChoicePropertyFactory(PropertyFactory *factory_, const MBXMLUtils::FQN xmlName_="", int mode_=1) : factory(factory_), xmlName(xmlName_), mode(mode_) { }
      PropertyInterface* createProperty(int i=0);
    protected:
      PropertyFactory *factory;
      MBXMLUtils::FQN xmlName;
      int mode;
  };

}

#endif

