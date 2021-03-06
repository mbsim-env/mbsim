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
#include "parameter_property_dialog.h"
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
      bool getEnabled() const override { return parent->getEnabled(); }
      void setParent(EmbedItemData* parent_) { parent = parent_; }
      EmbedItemData *getParent() const { return parent; }
      QString getReference() const override { return ""; }
      xercesc::DOMElement* getXMLElement() { return element; }
      void setXMLElement(xercesc::DOMElement *element_) { element = element_; }
      void removeXMLElements();
      static std::vector<Parameter*> createParameters(xercesc::DOMElement *element);
    protected:
      EmbedItemData *parent;
      xercesc::DOMElement *element;
  };

  class Parameter : public ParameterItem {
    public:
      Parameter() = default;
      QString getName() const override { return QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name")); }
      QString getValue() const override { return value; }
//      QString getValue() const override { return MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):""; }
      virtual MBXMLUtils::FQN getXMLType() const { return MBXMLUtils::PV%"Parameter"; }
      virtual xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      virtual PropertyDialog* createPropertyDialog() { return new ParameterPropertyDialog(this); }
      QMenu* createContextMenu() override { return new ParameterContextMenu(this); }
      virtual void updateValue() { }
    protected:
      QString value;
  };

  class StringParameter : public Parameter {
    public:
      StringParameter();
      MBXMLUtils::FQN getXMLType() const override { return MBXMLUtils::PV%"stringParameter"; }
      QString getType() const override { return "String parameter"; }
      PropertyDialog* createPropertyDialog() override { return new StringParameterPropertyDialog(this); }
      void updateValue() override;
  };

  class ScalarParameter : public Parameter {
    public:
      ScalarParameter();
      MBXMLUtils::FQN getXMLType() const override { return MBXMLUtils::PV%"scalarParameter"; }
      QString getType() const override { return "Scalar parameter"; }
      PropertyDialog* createPropertyDialog() override { return new ScalarParameterPropertyDialog(this); }
      void updateValue() override;
  };

  class VectorParameter : public Parameter {
    public:
      VectorParameter();
      MBXMLUtils::FQN getXMLType() const override { return MBXMLUtils::PV%"vectorParameter"; }
      QString getType() const override { return "Vector parameter"; }
      PropertyDialog* createPropertyDialog() override { return new VectorParameterPropertyDialog(this); }
      void updateValue() override;
  };

  class MatrixParameter : public Parameter {
    public:
      MatrixParameter();
      MBXMLUtils::FQN getXMLType() const override { return MBXMLUtils::PV%"matrixParameter"; }
      QString getType() const override { return "Matrix parameter"; }
      PropertyDialog* createPropertyDialog() override { return new MatrixParameterPropertyDialog(this); }
      void updateValue() override;
  };

  class ImportParameter : public Parameter {
    public:
      ImportParameter() = default;
      MBXMLUtils::FQN getXMLType() const override { return MBXMLUtils::PV%"import"; }
      QString getType() const override { return "Import"; }
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      PropertyDialog* createPropertyDialog() override { return new ImportParameterPropertyDialog(this); }
      void updateValue() override;
  };

  class Parameters : public ParameterItem {
    public:
      Parameters(EmbedItemData *parent);
      QString getName() const override { return parent->getName() + " parameters"; }
      QString getValue() const override { return ""; }
      QString getType() const override { return ""; }
      QMenu* createContextMenu() override { return new ParametersContextMenu(parent); }
      QString getReference() const override;
      bool hasReference() const override { return parent->hasParameterReference(); }
  };

}

#endif
