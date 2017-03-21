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
#include "mainwindow.h"
#include <QtGui/QFileDialog>
#include <xercesc/dom/DOMDocument.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
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

  void Parameters::addParameter(Parameter *param) {
    parameter.push_back(param);
    param->setParent(parent);
  }

  void Parameters::addParameters(const Parameters &list) {
    for(int i=0; i<list.getNumberOfParameters(); i++) {
      addParameter(list.getParameter(i));
      list.getParameter(i)->setParent(parent);
    }
  }

  void Parameters::removeParameter(Parameter *param) {  
    for (vector<Parameter*>::iterator it = parameter.begin(); it != parameter.end(); it++) {
      if(*it == param) {
        parameter.erase(it);
        delete param;
        return;
      }
    }
  }

  void Parameters::removeParameters() {
    for(size_t i=0; i<parameter.size(); i++)
      delete parameter[i];
  }

  Parameters Parameters::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Parameters param;
    param.initializeUsingXML(e);
    return param;
  }

  void Parameters::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(e);
      parameter->initializeUsingXML(e);
      addParameter(parameter);
      e=e->getNextElementSibling();
    }
  }

  DOMElement* Parameters::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getNodeType()==DOMNode::DOCUMENT_NODE ? static_cast<DOMDocument*>(parent) : parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(PV%"Parameter");
//    parent->insertBefore(ele0, NULL);
//    for(size_t i=0; i<parameter.size(); i++)
//      parameter[i]->writeXMLFile(ele0);
    return ele0;
  }

  void Parameters::writeXMLFile(const string &name) {
    shared_ptr<DOMDocument> doc=mw->parser->createDocument();
    writeXMLFile(doc.get());
    QFileInfo info(QString::fromStdString(name));
    QDir dir;
    if(!dir.exists(info.absolutePath()))
      dir.mkpath(info.absolutePath());
    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
  }

}
