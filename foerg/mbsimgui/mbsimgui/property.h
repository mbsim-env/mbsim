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

#ifndef _PROPERTIES_H_
#define _PROPERTIES_H_

#include <string>
#include <vector>
#include <cstdlib>
#include "treeitemdata.h"
#include "units.h"
#include "property_context_menu.h"
#include <boost/function.hpp>
#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

extern MBXMLUtils::NamespaceURI MBSIM;
extern MBXMLUtils::NamespaceURI PARAM;
extern MBXMLUtils::NamespaceURI OPENMBV;
extern MBXMLUtils::NamespaceURI MBSIMINT;
extern MBXMLUtils::NamespaceURI MBSIMCONTROL;

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
}

class QWidget;
class QMenu;
class Widget;
class WidgetFactory;
class PropertyPropertyDialog;

class PropertyInterface {
  public:
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) = 0;
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element) = 0;
    virtual void fromWidget(QWidget *widget) {}
    virtual void toWidget(QWidget *widget) {}
    virtual void initialize() {}
    virtual void deinitialize() {}
    virtual std::string getType() const {return "";}
};

class Property : public TreeItemData, public PropertyInterface {
  public:
    Property(const std::string &name_="", const std::string &value_="", const Units &units_=Units());
    virtual ~Property() {}
    virtual Property* clone() const {return 0;}
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element); 
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    const std::string& getName() const { return name; }
    const std::string& getValue() const { return value; }
    const std::string& getUnit() const { return unit; }
    const std::string& getEvaluation() const { return evaluation; }
    void setName(const std::string &data) { name = data; } 
    void setValue(const std::string &data) { value = data; }
    void setUnit(const std::string &data) { unit = data; } 
    void setEvaluation(const std::string &data) { evaluation = data; } 
    QMenu* createContextMenu() {return new PropertyContextMenu(this);}
    virtual PropertyPropertyDialog* createPropertyDialog() { return 0; }
    virtual PropertyPropertyDialog* createUnitDialog() { return 0; }
    virtual Widget* createWidget() { return 0; }
    virtual const Units& getUnits() const { return units; }
    virtual int getCurrentUnit() const { return -1; }
    virtual void setCurrentUnit(int i) { } 
    //bool isActive() const {return not(isDisabled());}
    void setDisabling(bool disabl_) {disabl=disabl_;}
    bool disabling() const {return disabl;}
    void setDisabled(bool disabled_);
    bool isDisabled() const {return disabled;}
    int getNumberOfProperties() const { return property.size(); }
    Property* getProperty(int i=0) const { return property[i]; }
    Property* setProperty(Property *property_, int i=0);
    void addProperty(Property *property_); 
    Property* getParent() const { return parent; }
    void setParent(Property *parent_);
    virtual void update() { if(signal) signal(); }
    boost::function<void()> signal, slot;
  protected:
    std::string name, value, unit, evaluation;
    Units units;
    bool disabl, disabled;
    std::vector<Property*> property;
    Property *parent;
};

class PhysicalProperty : public Property {
  public:
    PhysicalProperty(const std::string &name="", const std::string &value="", const Units &units=NoUnitUnits()) : Property(name,value,units) {
      if(units.getNumberOfUnits())
        setCurrentUnit(units.getDefaultUnit());
    }
    PhysicalProperty(const std::string &name, const std::string &value, const Units &units, int defaultUnit) : Property(name,value,units) {
      if(units.getNumberOfUnits())
        setCurrentUnit(defaultUnit);
    }
    void setValue(const std::string &data);
    void setUnits(const Units &units_) { units = units_; currentUnit=units.getDefaultUnit(); }
    void setCurrentUnit(int i) { currentUnit = i; setUnit(units.getUnit(currentUnit)); } 
    int getCurrentUnit() const { return currentUnit; }
    xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual int size(int i=0) { return 1; }
    virtual void setSize(int i=0, int size=0) { }
  protected:
    int currentUnit;
};

class PropertyFactory {
  public:
    virtual Property* createProperty(int i=0) = 0;
    virtual MBXMLUtils::FQN getName(int i=0) const { return ""; }
    virtual int getSize() const { return 0; }
    virtual WidgetFactory* createWidgetFactory() { return 0; }
};

#endif
