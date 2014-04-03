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

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class PropertyWidget;
  class PropertyDialog;
  class ExtWidget;
  class TextWidget;

  class Parameter : public TreeItemData {
    friend class ParameterPropertyDialog;
    public:
    Parameter(const std::string &name);
    virtual ~Parameter() {}
    virtual std::string getValue() const {return valuestr;}
    void setValue(const std::string &value) {valuestr = value;}
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual std::string getType() const { return "Parameter"; }
    const std::string& getName() const {return static_cast<const TextProperty*>(name.getProperty())->getText();}
    void setName(const std::string &str) {static_cast<TextProperty*>(name.getProperty())->setText(str);}
    virtual ParameterPropertyDialog* createPropertyDialog() {return new ParameterPropertyDialog(this);}
    virtual ParameterContextMenu* createContextMenu() {return new ParameterContextMenu;}
    protected:
    ExtProperty name, value;
    std::string valuestr;
  };

  class StringParameter : public Parameter {
    friend class StringParameterPropertyDialog;
    public:
    StringParameter(const std::string &name);
    virtual ~StringParameter() {}
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual std::string getType() const { return "stringParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new StringParameterPropertyDialog(this);}
  };

  class ScalarParameter : public Parameter {
    friend class ScalarParameterPropertyDialog;
    public:
    ScalarParameter(const std::string &name);
    virtual ~ScalarParameter() {}
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual std::string getType() const { return "scalarParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new ScalarParameterPropertyDialog(this);}
  };

  class VectorParameter : public Parameter {
    friend class VectorParameterPropertyDialog;
    public:
    VectorParameter(const std::string &name);
    virtual ~VectorParameter() {}
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual std::string getType() const { return "vectorParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new VectorParameterPropertyDialog(this);}
  };

  class MatrixParameter : public Parameter {
    friend class MatrixParameterPropertyDialog;
    public:
    MatrixParameter(const std::string &name);
    virtual ~MatrixParameter() {}
    virtual void initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual std::string getType() const { return "matrixParameter"; }
    virtual ParameterPropertyDialog* createPropertyDialog() {return new MatrixParameterPropertyDialog(this);}
  };

  class SearchPath : public Parameter {
    friend class SearchPathPropertyDialog;
    public:
      SearchPath(const std::string &name);
      virtual ~SearchPath() {}
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "searchPath"; }
  //    virtual ParameterPropertyDialog* createPropertyDialog() {return new MatrixParameterPropertyDialog(this);}
  };

  class ParameterList {
    public:
      bool readXMLFile(const std::string &filename);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element) const;
      int getSize() const {return name.size();}
      void addParameter(const std::string &name, const std::string &value, const std::string &type);
      void addParameterList(const ParameterList &list); 
      const std::string& getName(int i) const { return name[i]; }
      const std::string& getValue(int i) const { return value[i]; }
    private:
      std::vector<std::string> name, value, type;
  };

  class Parameters {
    protected:
      std::vector<Parameter*> parameter;
    public:
      void addParameter(Parameter *param) { parameter.push_back(param); }
      void addParameters(const Parameters &list); 
      void removeParameter(Parameter *param);
      void removeParameters();
      Parameter *getParameter(int i) const { return parameter[i]; }
      int getNumberOfParameters() const { return parameter.size(); }
      void initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      static Parameters readXMLFile(const std::string &filename);
      virtual void writeXMLFile(const std::string &name);
  };

}

#endif
