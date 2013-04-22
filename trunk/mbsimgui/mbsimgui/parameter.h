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

#ifndef _PARAMETER__H_
#define _PARAMETER__H_

#include "treeitemdata.h"
#include "basic_properties.h"
#include "extended_properties.h"
#include "parameter_property_dialog.h"
#include "parameter_context_menu.h"

class PropertyWidget;
class PropertyDialog;
class ExtWidget;
class TiXmlElement;
class TiXmlNode;
class TextWidget;

class Parameter : public TreeItemData {
  friend class ParameterPropertyDialog;
  public:
    Parameter(const std::string &name);
    virtual ~Parameter();
    std::string getValue() const;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    static Parameter* readXMLFile(const std::string &filename);
    virtual void writeXMLFile(const std::string &name);
    virtual void writeXMLFile() { writeXMLFile(getType()); }
    virtual std::string getType() const { return "matrixParameter"; }
    const std::string& getName() const {return static_cast<const TextProperty*>(name.getProperty())->getText();}
    void setName(const std::string &str) {static_cast<TextProperty*>(name.getProperty())->setText(str);}
    virtual ParameterPropertyDialog* createPropertyDialog() {return new ParameterPropertyDialog;}
    virtual ParameterContextMenu* createContextMenu() {return new ParameterContextMenu;}
  protected:
    ExtProperty name, value;
};

#endif
