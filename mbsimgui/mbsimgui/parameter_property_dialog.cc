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

#include <config.h>
#include "parameter_property_dialog.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "parameter.h"
#include "embeditemdata.h"
#include <QDialogButtonBox>
#include <QPushButton>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ParameterPropertyDialog::ParameterPropertyDialog(Parameter *parameter_) : PropertyDialog("Parameter Properties"), parameter(parameter_), name(nullptr) {
    addTab("General");
    if(not dynamic_cast<ImportParameter*>(parameter)) {
      name=new ExtWidget("Name",new TextWidget(parameter->getName()));
      addToTab("General",name);
    }

    addTab("Misc");
    vector<QString> list;
    list.emplace_back("normal");
    list.emplace_back("hidden");
    hidden=new QCheckBox("Hidden");
    hidden->setToolTip("Set this parameter hidden which prevents it "
                       "from appearing in the parameter tree. Hidden element will only appear if 'Show hidden element' "
                       "is enabled in the options.");
    addToTab("Misc",hidden);

    addTab("Comment");
    comment = new CommentWidget;
    addToTab("Comment", comment);
  }

  DOMElement* ParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    if(name) name->getWidget<TextWidget>()->setText(parameter->getName());
    comment->initializeUsingXML(parameter->getXMLElement());
    hidden->setChecked(E(parent)->getFirstProcessingInstructionChildNamed("MBSIMGUI_HIDDEN")!=nullptr);
    return parent;
  }

  DOMElement* ParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    parameter->removeXMLElements();
    if(name) E(parameter->getXMLElement())->setAttribute("name",name->getWidget<TextWidget>()->getText().toStdString());
    comment->writeXMLFile(parameter->getXMLElement(),ref);
    if(hidden->isChecked()) {
      DOMDocument *doc=parameter->getXMLElement()->getOwnerDocument();
      DOMProcessingInstruction *pi=doc->createProcessingInstruction(X()%"MBSIMGUI_HIDDEN", X()%"");
      parameter->getXMLElement()->insertBefore(pi, nullptr);
    }
    return nullptr;
  }

  void ParameterPropertyDialog::toWidget() {
    initializeUsingXML(parameter->getXMLElement());
  }

  void ParameterPropertyDialog::fromWidget() {
    writeXMLFile(parameter->getXMLElement());
    parameter->updateValue();
  }

  StringParameterPropertyDialog::StringParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    value = new ExtWidget("Value",new ExtStringWidget(dynamic_cast<Element*>(parameter->getParent())),false,false,"");
    addToTab("General", value);
  }

  DOMElement* StringParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* StringParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
    parameter->getParent()->updateName();
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(value->getWidget())->getWidget())->getValue());
    return nullptr;
  }

  ScalarParameterPropertyDialog::ScalarParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    value = new ExtWidget("Value",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,"");
    addToTab("General", value);
  }

  DOMElement* ScalarParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* ScalarParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(value->getWidget())->getWidget())->getValue());
    return nullptr;
  }

  VectorParameterPropertyDialog::VectorParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    value = new ExtWidget("Value",new ChoiceWidget(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,"");
    addToTab("General", value);
  }

  DOMElement* VectorParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* VectorParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(value->getWidget())->getWidget())->getValue());
    return nullptr;
  }

  MatrixParameterPropertyDialog::MatrixParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    value = new ExtWidget("Value",new ChoiceWidget(new MatRowsColsVarWidgetFactory(3,3),QBoxLayout::RightToLeft,5),false,false,"");
    addToTab("General", value);
  }

  DOMElement* MatrixParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* MatrixParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(value->getWidget())->getWidget())->getValue());
    return nullptr;
  }

  AnyParameterPropertyDialog::AnyParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    value = new ExtWidget("Value",new ExpressionWidget("0"));
    addToTab("General", value);
  }

  DOMElement* AnyParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* AnyParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
    return nullptr;
  }

  ImportParameterPropertyDialog::ImportParameterPropertyDialog(Parameter *parameter) : ParameterPropertyDialog(parameter) {
    //value = new ExtWidget("Value",new ExpressionWidget("0"));
    value = new ExtWidget("Value",new ChoiceWidget(new StringWidgetFactory("","\".\""),QBoxLayout::RightToLeft,5));
    addToTab("General", value);
  }

  DOMElement* ImportParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());
    value->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* ImportParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);
    value->writeXMLFile(parameter->getXMLElement(),ref);
//    parameter->setValue(static_cast<ExpressionWidget*>(value->getWidget())->getValue());
    return nullptr;
  }

}
