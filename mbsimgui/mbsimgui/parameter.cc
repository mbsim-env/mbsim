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
#include "parameter.h"
#include "objectfactory.h"
#include "utils.h"
#include "fileitemdata.h"
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  void ParameterItem::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  vector<Parameter*> ParameterItem::createParameters(DOMElement *element) {
    vector<Parameter*> param;
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(e);
      parameter->setXMLElement(e);
      parameter->updateValue();
      param.push_back(parameter);
      e=e->getNextElementSibling();
    }
    return param;
  }

  DOMElement* Parameter::createXMLElement(DOMNode *parent) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    E(element)->setAttribute("name", getXMLType().second);
    parent->insertBefore(element, nullptr);
    return element;
  }

  StringParameter::StringParameter() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"string.svg").string()));
  }

  void StringParameter::updateValue() {
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  ScalarParameter::ScalarParameter() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"scalar.svg").string()));
  }

  void ScalarParameter::updateValue() {
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  VectorParameter::VectorParameter() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"vector.svg").string()));
  }

  void VectorParameter::updateValue() {
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

  MatrixParameter::MatrixParameter() {
    icon = Utils::QIconCached(QString::fromStdString((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"matrix.svg").string()));
  }

  void MatrixParameter::updateValue() {
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

  DOMElement* ImportParameter::createXMLElement(DOMNode *parent) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    element=D(doc)->createElement(getXMLType());
    parent->insertBefore(element, nullptr);
    return element;
  }

  void ImportParameter::updateValue() {
    value = MBXMLUtils::E(element)->getFirstTextChild()?QString::fromStdString(MBXMLUtils::X()%MBXMLUtils::E(element)->getFirstTextChild()->getData()):"";
  }

  Parameters::Parameters(EmbedItemData *parent) : ParameterItem(parent) {
    icon = QIcon(new OverlayIconEngine((mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"container.svg").string(),
                                       (mw->getInstallPath()/"share"/"mbsimgui"/"icons"/"matrix.svg").string()));
  }

  QString Parameters::getReference() const {
    return parent->getDedicatedParameterFileItem()?parent->getDedicatedParameterFileItem()->getName():"";
  }

}
