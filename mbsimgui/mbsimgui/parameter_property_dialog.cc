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
#include "parameter_property_dialog.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "parameter.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ParameterPropertyDialog::ParameterPropertyDialog(Parameter *parameter_, QWidget *parent, Qt::WindowFlags f, bool readOnly) : PropertyDialog(parent,f), parameter(parameter_) {
    addTab("General");
    name=new ExtWidget("Name",new TextWidget(parameter->getName()),readOnly);
    addToTab("General",name);
  }

  DOMElement* ParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    static_cast<TextWidget*>(name->getWidget())->setText(parameter->getName());
    return parent;
  }

  DOMElement* ParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    parameter->removeXMLElements();
    E(parameter->getXMLElement())->setAttribute("name",static_cast<TextWidget*>(name->getWidget())->getText().toStdString());
    return NULL;
  }

  void ParameterPropertyDialog::toWidget(Parameter *parameter) {
    initializeUsingXML(parameter->getXMLElement());
  }

  void ParameterPropertyDialog::fromWidget(Parameter *parameter) {
    writeXMLFile(parameter->getXMLElement());
  }

  StringParameterPropertyDialog::StringParameterPropertyDialog(StringParameter *parameter, QWidget *parent, Qt::WindowFlags f) : ParameterPropertyDialog(parameter,parent,f) {
    value = new ExtWidget("Value",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,"");
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
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(value->getWidget())->getWidget())->getValue());
    return NULL;
  }

  ScalarParameterPropertyDialog::ScalarParameterPropertyDialog(ScalarParameter *parameter, QWidget *parent, Qt::WindowFlags f) : ParameterPropertyDialog(parameter,parent,f) {
    value = new ExtWidget("Value",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,"");
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
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(value->getWidget())->getWidget())->getValue());
    return NULL;
  }

  VectorParameterPropertyDialog::VectorParameterPropertyDialog(VectorParameter *parameter, QWidget *parent, Qt::WindowFlags f) : ParameterPropertyDialog(parameter,parent,f) {
    value = new ExtWidget("Value",new ChoiceWidget2(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,"");
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
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(value->getWidget())->getWidget())->getValue());
    return NULL;
  }

  MatrixParameterPropertyDialog::MatrixParameterPropertyDialog(MatrixParameter *parameter,QWidget *parent, Qt::WindowFlags f) : ParameterPropertyDialog(parameter,parent,f) {
    value = new ExtWidget("Value",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(3,3),QBoxLayout::RightToLeft,5),false,false,"");
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
//    parameter->setValue(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(value->getWidget())->getWidget())->getValue());
    return NULL;
  }

  ImportParameterPropertyDialog::ImportParameterPropertyDialog(ImportParameter *parameter,QWidget *parent, Qt::WindowFlags f) : ParameterPropertyDialog(parameter,parent,f,true) {
    value = new ExtWidget("Value",new ExpressionWidget("0"));
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
    return NULL;
  }

}
