/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _PARAMETER__H_
#define _PARAMETER__H_

#include "treeitemdata.h"
#include "parameter_context_menu.h"
#include "parameters_context_menu.h"
#include "embeditemdata.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class EmbedItemData;
  class PropertyWidget;
  class PropertyDialog;
  class ExtWidget;
  class TextWidget;

  class ParameterItem : public TreeItemData {
    public:
      ParameterItem(EmbedItemData *parent_=nullptr) : parent(parent_) { }
      QString getComment() const override { return ""; }
      bool getEnabled() const override { return parent->getEnabled(); }
      void setParent(EmbedItemData* parent_) { parent = parent_; }
      EmbedItemData *getParent() const { return parent; }
      QString getReference() const override { return ""; }
      xercesc::DOMElement* getXMLElement() override { return element; }
      void setXMLElement(xercesc::DOMElement *element_) { element = element_; }
      void removeXMLElements();
      static std::vector<Parameter*> createParameters(xercesc::DOMElement *element);
    protected:
      EmbedItemData *parent { nullptr };
      xercesc::DOMElement *element { nullptr };
  };

  class Parameter : public ParameterItem {
    MBSIMGUI_OBJECTFACTORY_CLASS(Parameter, ParameterItem, MBXMLUtils::PV%"Parameter", "Parameter");
    public:
      Parameter() = default;
      QString getName() const override { return name; }
      QString getValue() const override { return value; }
      QString getComment() const override { return comment; }
      void setComment(const QString &comment_) { comment = comment_; }
      void setValue(const QString &v) { value=v; }
      bool getHidden() const { return hidden; }
//      QString getValue() const override { return MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):""; }
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      virtual PropertyDialog* createPropertyDialog()=0;
      QMenu* createContextMenu() override { return new ParameterContextMenu(this); }
      virtual void updateValue(bool evaluate=false);
    protected:
      QString name, value, comment;
      bool hidden { false };
  };

  class StringParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(StringParameter, Parameter, MBXMLUtils::PV%"stringParameter", "String parameter");
    public:
      StringParameter();
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
  };

  class ScalarParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(ScalarParameter, Parameter, MBXMLUtils::PV%"scalarParameter", "Scalar parameter");
    public:
      ScalarParameter();
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
  };

  class VectorParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(VectorParameter, Parameter, MBXMLUtils::PV%"vectorParameter", "Vector parameter");
    public:
      VectorParameter();
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
  };

  class MatrixParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(MatrixParameter, Parameter, MBXMLUtils::PV%"matrixParameter", "Matrix parameter");
    public:
      MatrixParameter();
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
  };

  class AnyParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(AnyParameter, Parameter, MBXMLUtils::PV%"anyParameter", "Any parameter");
    public:
      AnyParameter();
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
  };

  class ImportParameter : public Parameter {
    MBSIMGUI_OBJECTFACTORY_CLASS(ImportParameter, Parameter, MBXMLUtils::PV%"import", "Import");
    public:
      ImportParameter();
      QString getName() const override { return name.isEmpty()?"<import without label>":"<"+name+">"; }
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      PropertyDialog* createPropertyDialog() override;
      void updateValue(bool evaluate=false) override;
    private:
      std::string action;
  };

  class Parameters;

  class ParameterEmbedItem : public ParameterItem {
    public:
      // this class is special -> do not use the define MBSIMGUI_OBJECTFACTORY_CLASS
      QString getType() const override { return getParent() ? getParent()->getType() : ""; }

      ParameterEmbedItem(EmbedItemData *parent);
      ~ParameterEmbedItem();
      QString getName() const override;
      QString getValue() const override { return ""; }
      QString getReference() const override;
      bool hasReference() const override { return parent->hasParameterReference(); }
      void setIcon(const QIcon &icon_) { icon = icon_; }
      void setParameters(Parameters *parameters_);
      Parameters* getParameters() { return parameters; }
      QMenu* createContextMenu() override;
    private:
      Parameters *parameters { nullptr };
  };

  class Parameters : public ParameterItem {
    public:
      // this class is special -> do not use the define MBSIMGUI_OBJECTFACTORY_CLASS
      QString getType() const override { return "Parameters"; }

      Parameters(EmbedItemData *parent);
      QString getName() const override { return "Parameters"; }
      QString getValue() const override { return ""; }
      QString getReference() const override { return ""; }
      bool hasReference() const override { return false; }
      QMenu* createContextMenu() override;
  };

}

#endif
