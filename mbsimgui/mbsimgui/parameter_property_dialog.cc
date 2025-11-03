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
#include "mainwindow.h"
#include "parameter_property_dialog.h"
#include "basic_widgets.h"
#include "treeitem.h"
#include "treemodel.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "project.h"
#include "xercesc/dom/DOMProcessingInstruction.hpp"
#include <QDialogButtonBox>
#include <QPushButton>
#include <evaluator/evaluator.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterPropertyDialog::ParameterPropertyDialog(Parameter *parameter_) : PropertyDialog("Parameter Properties"), parameter(parameter_), name(nullptr), npl(mw->eval) {
    mw->updateParameters(parameter->getParent(), parameter, false);

    addTab("General");
    if(not dynamic_cast<ImportParameter*>(parameter)) {
      name=new ExtWidget("Name",new TextWidget(parameter->getName()));
      addToTab("General",name);
    }

    addTab("Comment");
    comment = new CommentWidget;
    addToTab("Comment", comment);

    addTab("Misc");
    hidden = new ExtWidget("Hidden",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5));
    hidden->setToolTip("Set this parameter hidden which prevents it "
                       "from appearing in the parameter tree. Hidden element will only appear if 'Show hidden element' "
                       "is enabled in the options.");
    addToTab("Misc",hidden);

    mbsimguiContextAction = new MBSimGUIContextAction;
    addToTab("Misc", mbsimguiContextAction);
  }

  DOMElement* ParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    if(name) name->getWidget<TextWidget>()->setText(parameter->getName());
    comment->initializeUsingXML(parameter->getXMLElement());

    auto *pi=E(parent)->getFirstProcessingInstructionChildNamed("MBSIMGUI_HIDDEN");
    hidden->setActive(pi!=nullptr);
    if(pi) {
      auto value=X()%pi->getData();
      hidden->getWidget<ChoiceWidget>()->setIndex(value.empty()?0:1);
      if(value.empty())
        hidden->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->setValue("1");
      else
        hidden->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->setValue(value.c_str());
    }

    mbsimguiContextAction->initializeUsingXML(parameter->getXMLElement());
    return parent;
  }

  DOMElement* ParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    parameter->removeXMLElements();
    if(name) E(parameter->getXMLElement())->setAttribute("name",name->getWidget<TextWidget>()->getText().toStdString());
    comment->writeXMLFile(parameter->getXMLElement(),ref);

    if(hidden->isActive()) {
      if(hidden->getWidget<ChoiceWidget>()->getIndex()==0) {
        if(hidden->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->getValue()!=mw->getProject()->getVarFalse())
          E(parameter->getXMLElement())->addProcessingInstructionChildNamed("MBSIMGUI_HIDDEN", "");
      }
      else
        E(parameter->getXMLElement())->addProcessingInstructionChildNamed("MBSIMGUI_HIDDEN",
            hidden->getWidget<ChoiceWidget>()->getWidget<PhysicalVariableWidget>()->getValue().toStdString());
    }

    mbsimguiContextAction->writeXMLFile(parameter->getXMLElement(),ref);
    return nullptr;
  }

  void ParameterPropertyDialog::toWidget() {
    initializeUsingXML(parameter->getXMLElement());
  }

  void ParameterPropertyDialog::fromWidget() {
    writeXMLFile(parameter->getXMLElement());
    parameter->updateValue();
  }

  void ParameterPropertyDialog::showEvent(QShowEvent *ev) {
    mw->setCurrentlyEditedItem(parameter);
    PropertyDialog::showEvent(event);
  }

  void ParameterPropertyDialog::hideEvent(QHideEvent *ev) {
    mw->setCurrentlyEditedItem(nullptr);
    PropertyDialog::hideEvent(event);
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
    value = new ExtWidget("Value",new PhysicalVariableWidget(new ExpressionWidget("0", 1), {}, 0, true));
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
    auto pname=parameter->getName();
    label=new ExtWidget("Label",new TextWidget(pname=="<import without label>"?"":pname.mid(1, pname.length()-2)));
    addToTab("General",label);

    bool hidden = false;
    for(auto &x : Evaluator::getImportActions())
      actionList.emplace_back(pair<QString,QString>{get<0>(x).c_str(), get<2>(x).c_str()});
    int defaultIdx = Evaluator::getImportActionDefaultIdx();
    if(Evaluator::getImportActionOnlyOneNoneDepr()>=0)
      hidden = true;
    vector<QString> list;
    transform(actionList.begin(), actionList.end(), back_insert_iterator(list), [](const auto& x){ return x.second; });
    action = new ExtWidget("Action",new TextChoiceWidget(list,defaultIdx,false));
    action->setHidden(hidden);
    addToTab("General", action);

    auto valueW=new TextEditorWidget();
    value = new ExtWidget("Value",valueW);
    valueW->enableSyntaxHighlighter();
    addToTab("General", value);
  }

  DOMElement* ImportParameterPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ParameterPropertyDialog::initializeUsingXML(parameter->getXMLElement());

    auto pname=parameter->getName();
    label->getWidget<TextWidget>()->setText(pname=="<import without label>"?"":pname.mid(1, pname.length()-2));

    bool hidden = false;
    auto xmlAction = E(parameter->getXMLElement())->getAttribute("action");
    int actionIdx = 0;
    if(!xmlAction.empty()) {
      auto it = find_if(actionList.begin(), actionList.end(), [&xmlAction](const auto &x){ return x.first.toStdString() == xmlAction; });
      actionIdx = distance(actionList.begin(), it);
    }
    if(Evaluator::getImportActions().size()==1) // only one option available -> do not show at all
      hidden = true;
    auto ond=Evaluator::getImportActionOnlyOneNoneDepr();
    if(ond>=0 && actionIdx==ond)
      hidden = true;
    action->getWidget<TextChoiceWidget>()->setCurrentIndex(actionIdx);
    action->setHidden(hidden);

    value->initializeUsingXML(parameter->getXMLElement());

    return parent;
  }

  DOMElement* ImportParameterPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ParameterPropertyDialog::writeXMLFile(parameter->getXMLElement(),ref);

    if(auto l=label->getWidget<TextWidget>()->getText(); !l.isEmpty())
      E(parameter->getXMLElement())->setAttribute("label",l.toStdString());

    value->writeXMLFile(parameter->getXMLElement(),ref);

    optional<QString> actionStr;
    auto it=actionList.begin()+action->getWidget<TextChoiceWidget>()->getCurrentIndex();
    if(!it->first.isEmpty())
      actionStr=it->first;
    if(actionStr)
      E(static_cast<DOMElement*>(parent))->setAttribute("action", actionStr.value().toStdString());
    else
      E(static_cast<DOMElement*>(parent))->removeAttribute("action");

    return nullptr;
  }

}
