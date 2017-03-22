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
#include <QtGui/QFileDialog>
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

extern DOMLSParser *parser;

namespace MBSimGUI {

  extern QDir mbsDir;
  extern bool absolutePath;

  Parameter::Parameter(const string &name_) : parent(NULL), name(name_), config(false) {
  }

  void Parameter::initializeUsingXML(DOMElement *element) {
    this->element = element;
    setName(E(element)->getAttribute("name"));
    setValue(X()%E(element)->getFirstTextChild()->getData());
    config = true;
  }

  DOMElement* Parameter::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(PV%getType());
//    E(ele0)->setAttribute("name", getName());
//    parent->insertBefore(ele0, NULL);
    return ele0;
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
    element=D(doc)->createElement(PV%getType());
    E(element)->setAttribute("name", getName());
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

  vector<Parameter*> Parameter::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc(parser->parseURI(X()%filename));
    DOMElement *e=doc->getDocumentElement();
    return Parameter::initializeParametersUsingXML(e);
  }

  StringParameter::StringParameter(const string &name) : Parameter(name) {
  }

  ScalarParameter::ScalarParameter(const string &name, const string &value_) : Parameter(name) {
  }

  VectorParameter::VectorParameter(const string &name) : Parameter(name) {
  }

  MatrixParameter::MatrixParameter(const string &name) : Parameter(name) {
  }

  ImportParameter::ImportParameter() : Parameter("import") {
  }

}
