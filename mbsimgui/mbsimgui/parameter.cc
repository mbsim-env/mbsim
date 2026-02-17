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
#include <xercesc/dom/DOMNamedNodeMap.hpp>
#include <xercesc/dom/DOMAttr.hpp>
#include <xercesc/dom/DOMComment.hpp>
#include "parameter.h"
#include "objectfactory.h"
#include "parameter_property_dialog.h"
#include "parameter_embed_item_context_menu.h"
#include "parameter_view.h"
#include "utils.h"
#include "fileitemdata.h"
#include "mainwindow.h"
#include "xercesc/dom/DOMProcessingInstruction.hpp"
#include "dialogs.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace fmatvec;

namespace MBSimGUI {

  extern MainWindow *mw;

  MBSIMGUI_REGOBJECTFACTORY(StringParameter);
  MBSIMGUI_REGOBJECTFACTORY(ScalarParameter);
  MBSIMGUI_REGOBJECTFACTORY(VectorParameter);
  MBSIMGUI_REGOBJECTFACTORY(MatrixParameter);
  MBSIMGUI_REGOBJECTFACTORY(AnyParameter);
  MBSIMGUI_REGOBJECTFACTORY(ImportParameter);

  void ParameterItem::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
    while(element->getAttributes()->getLength()>0)
      element->removeAttributeNode(static_cast<DOMAttr*>(element->getAttributes()->item(0)));
  }

  vector<Parameter*> ParameterItem::createParameters(DOMElement *element) {
    vector<Parameter*> param;
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      Parameter *parameter=ObjectFactory::getInstance().create<Parameter>(e);
      parameter->setXMLElement(e);
      parameter->updateValue();
      param.push_back(parameter);
      e=e->getNextElementSibling();
    }
    return param;
  }

  DOMElement* Parameter::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    E(element)->setAttribute("name", getXMLType().second);
    parent->insertBefore(element, nullptr);
    return element;
  }

  void Parameter::updateValue(bool evaluate) {
    if(evaluate && getParent()) {
      hidden=false;
      if(auto pi = E(element)->getFirstProcessingInstructionChildNamed("MBSIMGUI_HIDDEN"); pi) {
        if(X()%pi->getData()=="")
          hidden=true;
        else {
          // update also values which need a evaluation using the current evaluator
          NewParamLevel npl(mw->eval);
          mw->updateParameters(this->getParent(), this, true);
          try {
            hidden=mw->eval->cast<int>(mw->eval->eval(X()%pi->getData(), getXMLElement()));
          }
          catch(std::exception &ex) {
            auto msg = dynamic_cast<DOMEvalException*>(&ex) ? static_cast<DOMEvalException&>(ex).getMessage() : ex.what();
            mw->statusBar()->showMessage(("Unable to evaluate hidden flag: " + msg).c_str());
            cerr << "Enable to evaluate hidden flag: " << ex.what() << endl;
            if(!mw->getErrorOccured()) {
              auto diag = new HiddenParErrorDialog(mw,
                EmbedDOMLocator::convertToRootHRXPathExpression(E(getXMLElement())->getRootXPathExpression()).c_str(),
                ex.what());
              diag->exec();
            }
            mw->setErrorOccured();
          }
          catch(...) {
            mw->statusBar()->showMessage("Unknown exception");
            cerr << "Unknown exception" << endl;
            if(!mw->getErrorOccured()) {
              auto diag = new HiddenParErrorDialog(mw,
                EmbedDOMLocator::convertToRootHRXPathExpression(E(getXMLElement())->getRootXPathExpression()).c_str(),
                "Unknown exception");
              diag->exec();
            }
            mw->setErrorOccured();
          }
        }
      }
      auto index = getModelIndex();
      QSettings settings;
      auto showHiddenElements = settings.value("mainwindow/options/showhiddenelements", true).toBool();
      mw->getParameterView()->setRowHidden(index.row(), index.parent(), hidden && !showHiddenElements);
    }

    name = QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name"));
    auto *cele = E(element)->getFirstCommentChild();
    if(cele)
      comment = QString::fromStdString(X()%cele->getNodeValue());
    else
      comment.clear();
    if(orgIcon.isNull())
      orgIcon=icon;
    if(E(element)->getFirstProcessingInstructionChildNamed("MBSIMGUI_CONTEXT_ACTION")!=nullptr)
      icon = QIcon(new OverlayIconEngine(orgIcon,
        Utils::QIconCached((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"contextactionoverlay.svg").string().c_str())));
    else
      icon = orgIcon;
  }

  PropertyDialog* StringParameter::createPropertyDialog() {
    return new StringParameterPropertyDialog(this);
  }

  StringParameter::StringParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"string.svg").string()));
  }

  void StringParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  PropertyDialog* ScalarParameter::createPropertyDialog() {
    return new ScalarParameterPropertyDialog(this);
  }

  ScalarParameter::ScalarParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"scalar.svg").string()));
  }

  void ScalarParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  PropertyDialog* VectorParameter::createPropertyDialog() {
    return new VectorParameterPropertyDialog(this);
  }

  VectorParameter::VectorParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"vector.svg").string()));
  }

  void VectorParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    DOMElement *ele=element->getFirstElementChild();
    if(ele and E(ele)->getTagName() == PV%"xmlVector") {
      vector<QString> v;
      DOMElement *ei=ele->getFirstElementChild();
      while(ei and E(ei)->getTagName()==PV%"ele") {
	v.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
	ei=ei->getNextElementSibling();
      }
      value = toQStr(v);
    }
    else if(ele and E(ele)->getTagName() == PV%"fromFile")
      value = QString::fromStdString(MBXMLUtils::E(ele)->getAttribute("href"));
    else if(MBXMLUtils::E(element)->getFirstTextChild())
      value = QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData());
  }

  PropertyDialog* MatrixParameter::createPropertyDialog() {
    return new MatrixParameterPropertyDialog(this);
  }

  MatrixParameter::MatrixParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"matrix.svg").string()));
  }

  void MatrixParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    DOMElement *ele=element->getFirstElementChild();
    if(ele and E(ele)->getTagName() == PV%"xmlMatrix") {
      vector<vector<QString>> m;
      DOMElement *ei=ele->getFirstElementChild();
      while(ei and E(ei)->getTagName()==PV%"row") {
	DOMElement *ej=ei->getFirstElementChild();
	m.emplace_back();
	while(ej and E(ej)->getTagName()==PV%"ele") {
	  m[m.size()-1].push_back(QString::fromStdString(X()%E(ej)->getFirstTextChild()->getData()));
	  ej=ej->getNextElementSibling();
	}
	ei=ei->getNextElementSibling();
      }
      value = toQStr(m);
    }
    else if(ele and E(ele)->getTagName() == PV%"fromFile")
      value = QString::fromStdString(MBXMLUtils::E(ele)->getAttribute("href"));
    else if(MBXMLUtils::E(element)->getFirstTextChild())
      value = QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData());
  }

  PropertyDialog* AnyParameter::createPropertyDialog() {
    return new AnyParameterPropertyDialog(this);
  }

  AnyParameter::AnyParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"any.svg").string()));
  }

  void AnyParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  PropertyDialog* ImportParameter::createPropertyDialog() {
    return new ImportParameterPropertyDialog(this);
  }

  ImportParameter::ImportParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"import.svg").string()));
  }

  DOMElement* ImportParameter::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    parent->insertBefore(element, nullptr);
    return element;
  }

  void ImportParameter::updateValue(bool evaluate) {
    Parameter::updateValue(evaluate);
    name = QString::fromStdString(MBXMLUtils::E(element)->getAttribute("label"));
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
    action = E(element)->getAttribute("action");
  }

  ParameterEmbedItem::ParameterEmbedItem(EmbedItemData *parent) : ParameterItem(parent) {
    // icon is set by the element which has created the parameter during it ctor
  }

  ParameterEmbedItem::~ParameterEmbedItem() {
    delete parameters;
  }

  void ParameterEmbedItem::setParameters(Parameters *parameters_) {
    delete parameters;
    parameters = parameters_;
  }

  QString ParameterEmbedItem::getReference() const {
    return parent->getDedicatedParameterFileItem()?parent->getDedicatedParameterFileItem()->getName():"";
  }

  QString ParameterEmbedItem::getName() const {
    return parent->getName();
  }

  QMenu* ParameterEmbedItem::createContextMenu() {
    auto *e=parent->getXMLElement();
    if(!e)
      return nullptr;
    if(E(e)->getTagName()==PV%"Embed") // unhandled Embed in mbsimgui
      return nullptr;
    return new ParameterEmbedItemContextMenu(parent);
  }

  Parameters::Parameters(EmbedItemData *parent): ParameterItem(parent) {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"container.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"matrix.svg").string()));
  }

  QMenu* Parameters::createContextMenu() {
    auto *e=parent->getXMLElement();
    if(!e)
      return nullptr;
    if(E(e)->getTagName()==PV%"Embed") // unhandled Embed in mbsimgui
      return nullptr;
    return new ParametersContextMenu(parent);
  }

}
