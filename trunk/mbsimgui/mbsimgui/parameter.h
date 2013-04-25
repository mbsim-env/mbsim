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
    virtual std::string getValue() const = 0;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "Parameter"; }
    const std::string& getName() const {return static_cast<const TextProperty*>(name.getProperty())->getText();}
    void setName(const std::string &str) {static_cast<TextProperty*>(name.getProperty())->setText(str);}
    virtual ParameterPropertyDialog* createPropertyDialog() {return new ParameterPropertyDialog;}
    virtual ParameterContextMenu* createContextMenu() {return new ParameterContextMenu;}
  protected:
    ExtProperty name;
};

class ScalarParameter : public Parameter {
  friend class ScalarParameterPropertyDialog;
  public:
    ScalarParameter(const std::string &name);
    virtual ~ScalarParameter();
    std::string getValue() const;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "scalarParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new ScalarParameterPropertyDialog;}
  protected:
    ExtProperty value;
};

class VectorParameter : public Parameter {
  friend class VectorParameterPropertyDialog;
  public:
    VectorParameter(const std::string &name);
    virtual ~VectorParameter();
    std::string getValue() const;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "vectorParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new VectorParameterPropertyDialog;}
  protected:
    ExtProperty value;
};

class MatrixParameter : public Parameter {
  friend class MatrixParameterPropertyDialog;
  public:
    MatrixParameter(const std::string &name);
    virtual ~MatrixParameter();
    std::string getValue() const;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual std::string getType() const { return "matrixParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new MatrixParameterPropertyDialog;}
  protected:
    ExtProperty value;
};

class ParameterList {
  public:
    void readXMLFile(const std::string &filename);
    //virtual void writeXMLFile(const std::string &name);
    int getSize() const {return name.size();}
    void addParameter(const std::string &name_, const std::string &value_) {name.push_back(name_); value.push_back(value_);}
    void addParameterList(const ParameterList &list); 
    const std::string& getParameterName(int i) const {return name[i];}
    const std::string& getParameterValue(int i) const {return value[i];}
    //void printList() const;
    //void clear() {name.clear(); value.clear();}
      private:
    std::vector<std::string> name, value;
};

#endif
