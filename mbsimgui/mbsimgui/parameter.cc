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
#include "parameter.h"
#include "objectfactory.h"
#include "utils.h"
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Parameter::Parameter(const QString &name_) : parent(NULL), name(name_), config(false) {
  }

  void Parameter::initializeUsingXML(DOMElement *element) {
    this->element = element;
    setName(QString::fromStdString(E(element)->getAttribute("name")));
    config = true;
  }

  void Parameter::removeXMLElements() {
    DOMNode *e = element->getFirstChild();
    while(e) {
      DOMNode *en=e->getNextSibling();
      element->removeChild(e);
      e = en;
    }
  }

  DOMElement* Parameter::createXMLElement(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
//    static_cast<DOMElement*>(parent)->getTagName()
    DOMElement* embed =  static_cast<DOMElement*>(parent->getParentNode());
    if(X()%embed->getNodeName()!="Embed") {
      DOMElement *ele=D(doc)->createElement(PV%"Embed");
      embed->insertBefore(ele,parent);
      embed = ele;
      ele=D(doc)->createElement(PV%"Parameter");
      embed->insertBefore(ele,NULL);
      embed->insertBefore(parent,NULL);
    }
    else if(X()%embed->getFirstElementChild()->getNodeName()!="Parameter") {
      DOMElement *ele=D(doc)->createElement(PV%"Parameter");
      embed->insertBefore(ele,embed->getFirstElementChild());
    }
    element=D(doc)->createElement(PV%getType().toStdString());
    E(element)->setAttribute("name", getName().toStdString());
    embed->getFirstElementChild()->insertBefore(element,NULL);
    return element;
  }

  vector<Parameter*> Parameter::initializeParametersUsingXML(DOMElement *element) {
    vector<Parameter*> param;
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(e);
      parameter->initializeUsingXML(e);
      param.push_back(parameter);
      e=e->getNextElementSibling();
    }
    return param;
  }

  StringParameter::StringParameter(const QString &name) : Parameter(name) {
  }

  void StringParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    setValue(QString::fromStdString(X()%E(element)->getFirstTextChild()->getData()));
  }

  ScalarParameter::ScalarParameter(const QString &name, const QString &value_) : Parameter(name) {
  }

  void ScalarParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    setValue(QString::fromStdString(X()%E(element)->getFirstTextChild()->getData()));
  }

  VectorParameter::VectorParameter(const QString &name) : Parameter(name) {
  }

  void VectorParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    DOMElement *ele=element->getFirstElementChild();
    if(ele) {
      if(E(ele)->getTagName() == PV%"xmlVector") {
        DOMElement *ei=ele->getFirstElementChild();
        vector<QString> value;
        while(ei && E(ei)->getTagName()==PV%"ele") {
          value.push_back(QString::fromStdString(X()%E(ei)->getFirstTextChild()->getData()));
          ei=ei->getNextElementSibling();
        }
        setValue(toQStr<QString>(value));
      }
      else if(E(ele)->getTagName() == (PV%"fromFile"))
        setValue(QString::fromStdString((E(ele)->getAttribute("href"))));
    }
    else {
      DOMText *text=E(element)->getFirstTextChild();
      if(text)
      setValue(QString::fromStdString(X()%text->getData()));
    }
  }

  MatrixParameter::MatrixParameter(const QString &name) : Parameter(name) {
  }

  void MatrixParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    DOMElement *ele=element->getFirstElementChild();
    if(ele) {
      if(E(ele)->getTagName() == PV%"xmlMatrix") {
        DOMElement *ei=ele->getFirstElementChild();
        vector<vector<QString> > value;
        while(ei && E(ei)->getTagName()==PV%"row") {
          DOMElement *ej=ei->getFirstElementChild();
          value.push_back(vector<QString>());
          while(ej && E(ej)->getTagName()==PV%"ele") {
            value[value.size()-1].push_back(QString::fromStdString(X()%E(ej)->getFirstTextChild()->getData()));
            ej=ej->getNextElementSibling();
          }
          ei=ei->getNextElementSibling();
        }
        setValue(toQStr<QString>(value));
      }
      else if(E(ele)->getTagName() == (PV%"fromFile"))
        setValue(QString::fromStdString((E(ele)->getAttribute("href"))));
    }
    else {
      DOMText *text=E(element)->getFirstTextChild();
      if(text)
      setValue(QString::fromStdString(X()%text->getData()));
    }
  }

  ImportParameter::ImportParameter() : Parameter("import") {
  }

}
