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
#include "parameter_view.h"
#include "utils.h"
#include "fileitemdata.h"
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

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

  void Parameter::updateValue() {
    hidden = E(element)->getFirstProcessingInstructionChildNamed("MBSIMGUI_HIDDEN")!=nullptr;
    QSettings settings;
    bool showHiddenElements=settings.value("mainwindow/options/showhiddenelements", false).toBool();
    if(getModelIndex().isValid())
      mw->getParameterView()->setRowHidden(getModelIndex().row(), getModelIndex().parent(), hidden && !showHiddenElements);
    name = QString::fromStdString(MBXMLUtils::E(element)->getAttribute("name"));
    auto *cele = E(element)->getFirstCommentChild();
    if(cele)
      comment = QString::fromStdString(X()%cele->getNodeValue());
    else
      comment.clear();
  }

  PropertyDialog* StringParameter::createPropertyDialog() {
    return new StringParameterPropertyDialog(this);
  }

  StringParameter::StringParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"string.svg").string()));
  }

  void StringParameter::updateValue() {
    Parameter::updateValue();
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  PropertyDialog* ScalarParameter::createPropertyDialog() {
    return new ScalarParameterPropertyDialog(this);
  }

  ScalarParameter::ScalarParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"scalar.svg").string()));
  }

  void ScalarParameter::updateValue() {
    Parameter::updateValue();
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  PropertyDialog* VectorParameter::createPropertyDialog() {
    return new VectorParameterPropertyDialog(this);
  }

  VectorParameter::VectorParameter() {
    icon = Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"vector.svg").string()));
  }

  void VectorParameter::updateValue() {
    Parameter::updateValue();
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

  void MatrixParameter::updateValue() {
    Parameter::updateValue();
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

  void AnyParameter::updateValue() {
    Parameter::updateValue();
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

  void ImportParameter::updateValue() {
    Parameter::updateValue();
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
    action = E(element)->getAttribute("action");
  }

  Parameters::Parameters(EmbedItemData *parent) : ParameterItem(parent) {
    icon = QIcon(new OverlayIconEngine((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"container.svg").string(),
                                       (MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"matrix.svg").string()));
  }

  QString Parameters::getReference() const {
    return parent->getDedicatedParameterFileItem()?parent->getDedicatedParameterFileItem()->getName():"";
  }

  QMenu* Parameters::createContextMenu() {
    if(E(parent->getXMLElement())->getTagName()==PV%"Embed") // unhandled Embed in mbsimgui
      return nullptr;
    return new ParametersContextMenu(parent);
  }

}
