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
#include "basic_properties.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include <QtGui/QFileDialog>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;
  extern QDir mbsDir;
  extern bool absolutePath;

  Parameter::Parameter(const string &name__, Element *parent_) : parent(parent_), name_(name__) {
    name.setProperty(new TextProperty(name_,""));
  }

  void Parameter::setName(const std::string &str) {
    name_ = str;
//    static_cast<TextProperty*>(name.getProperty())->setText(str);
    E(element)->setAttribute("name", str);
  }

  void Parameter::initializeUsingXML(DOMElement *element) {
    this->element = element;
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
    element=D(doc)->createElement(PV%getType());
    E(element)->setAttribute("name", getName());
    embed->getFirstElementChild()->insertBefore(element,NULL);
    return element;
  }

  StringParameter::StringParameter(const string &name, Element *parent) : Parameter(name,parent) {
    value.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("'string'","",vector<string>(2,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  void StringParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  DOMElement* StringParameter::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Parameter::writeXMLFile(parent);
    value.writeXMLFile(ele0);
    return ele0;
  }

  ScalarParameter::ScalarParameter(const string &name, Element *parent, const string &value_) : Parameter(name,parent) {

    value.setProperty(new ChoiceProperty2(new ScalarPropertyFactory(value_,"",vector<string>(2,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  DOMElement* ScalarParameter::createXMLElement(DOMNode *parent) {
    Parameter::createXMLElement(parent);
    DOMDocument *doc=parent->getOwnerDocument();
    DOMText *text = doc->createTextNode(X()%"0");
    element->insertBefore(text, NULL);
    return element;
  }

  void ScalarParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  DOMElement* ScalarParameter::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Parameter::writeXMLFile(parent);
    value.writeXMLFile(ele0);
    return ele0;
  }

  VectorParameter::VectorParameter(const string &name, Element *parent) : Parameter(name,parent) {

    value.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,"",vector<string>(3,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  void VectorParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  DOMElement* VectorParameter::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Parameter::writeXMLFile(parent);
    value.writeXMLFile(ele0);
    return ele0;
  }

  MatrixParameter::MatrixParameter(const string &name, Element *parent) : Parameter(name,parent) {

    value.setProperty(new ChoiceProperty2(new MatPropertyFactory(getScalars<string>(3,3,"0"),"",vector<string>(3,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  void MatrixParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }

  DOMElement* MatrixParameter::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Parameter::writeXMLFile(parent);
    value.writeXMLFile(ele0);
    return ele0;
  }

  ImportParameter::ImportParameter(Element *parent) : Parameter("import",parent) {
    value.setProperty(new ExpressionProperty("'.'"));
    setValue(static_cast<ExpressionProperty*>(value.getProperty())->getValue());
  }

  void ImportParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<ExpressionProperty*>(value.getProperty())->getValue());
  }
  
  DOMElement* ImportParameter::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(PV%getType());
//    parent->insertBefore(ele0, NULL);
//    value.writeXMLFile(ele0);
    return ele0;
  }

  void Parameters::addParameters(const Parameters &list) {
    for(int i=0; i<list.getNumberOfParameters(); i++)
      addParameter(list.getParameter(i));
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
    Parameters param(NULL);
    param.initializeUsingXML(e);
    return param;
  }

  void Parameters::initializeUsingXML(DOMElement *element) {
    DOMElement *e=element->getFirstElementChild();
    while(e) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(e,parent);
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
