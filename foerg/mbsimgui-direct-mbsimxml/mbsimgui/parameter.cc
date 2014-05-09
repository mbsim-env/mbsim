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
using namespace boost;

namespace MBSimGUI {

  extern QDir mbsDir;
  extern bool absolutePath;

  Parameter::Parameter(const string &name_) {
    name.setProperty(new TextProperty(name_,""));
  }

  //return static_cast<const ExtPhysicalVarProperty*>(value.getProperty())->getValue();

  void Parameter::initializeUsingXML(DOMElement *element) {
  }

  DOMElement* Parameter::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(PARAM%getType());
    E(ele0)->setAttribute("name", getName());
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  StringParameter::StringParameter(const string &name) : Parameter(name) {

   value.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0","",vector<string>(2,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
//   value.setProperty(new TextProperty("0","",true));
//    setValue(static_cast<const TextProperty*>(value.getProperty())->getText());
  }

  void StringParameter::initializeUsingXML(DOMElement *element) {
    Parameter::initializeUsingXML(element);
    value.initializeUsingXML(element);
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
//    Parameter::initializeUsingXML(element);
//    TextProperty *val = static_cast<TextProperty*>(value.getProperty());
//    val->initializeUsingXML(element);
//    setValue(val->getText());
  }

  DOMElement* StringParameter::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Parameter::writeXMLFile(parent);
    value.writeXMLFile(ele0);
    return ele0;
//    DOMElement *ele0 = Parameter::writeXMLFile(parent);
//    TextProperty *val = static_cast<TextProperty*>(value.getProperty());
//    val->writeXMLFile(ele0);
//    return ele0;
  }

  ScalarParameter::ScalarParameter(const string &name) : Parameter(name) {

    value.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0","",vector<string>(2,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
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

  VectorParameter::VectorParameter(const string &name) : Parameter(name) {

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

  MatrixParameter::MatrixParameter(const string &name) : Parameter(name) {

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

  SearchPathParameter::SearchPathParameter(const string &name) : Parameter("searchPath") {
    value.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0","",vector<string>(2,"")),"",5));
    setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());
  }
  
  void SearchPathParameter::initializeUsingXML(DOMElement *element) {
   // Parameter::initializeUsingXML(element);
   // value.initializeUsingXML(element);
   // setValue(static_cast<PhysicalVariableProperty*>(static_cast<ChoiceProperty2*>(value.getProperty())->getProperty())->getValue());

     string value = E(element)->getAttribute("href"); 
//     setValue(mbsDir.absoluteFilePath(QString::fromStdString(value)).toStdString());
//     setValue(mbsDir.relativeFilePath(QString::fromStdString(value)).toStdString());
     setValue(value);
//     cout << getValue() << endl;
//     cout << MBXMLUtils::OctEval::cast<string>(MainWindow::octEval->stringToOctValue("\'"+getValue()+"\'")) << endl;
  }
  
  DOMElement* SearchPathParameter::writeXMLFile(DOMNode *parent) {
    //DOMElement *ele0 = Parameter::writeXMLFile(parent);
    //value.writeXMLFile(ele0);
    //return ele0;

    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(PARAM%getType());
    parent->insertBefore(ele0, NULL);
    string relFileName = absolutePath?getValue():mbsDir.relativeFilePath(QString::fromStdString(getValue())).toStdString();
    relFileName = getValue();
    cout << relFileName << endl;
    E(ele0)->setAttribute("href", relFileName);
  //  ExtPhysicalVarProperty *val = static_cast<ExtPhysicalVarProperty*>(value.getProperty());
  //  val->writeXMLFile(ele0);
    return ele0;
  }

  void ParameterList::addParameter(const string &name_, const string &value_, const string &type_) {
    name.push_back(name_); 
    value.push_back(value_);
    type.push_back(type_);
  }

  void ParameterList::addParameterList(const ParameterList &list) {
    for(int i=0; i<list.getSize(); i++)
      addParameter(list.name[i],list.value[i],list.type[i]);
  }

  bool ParameterList::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    DOMElement *E=e->getFirstElementChild();
    vector<QString> refFrame;
    while(E) {
      Parameter *parameter=ObjectFactory::getInstance()->createParameter(E);
      parameter->initializeUsingXML(E);
      addParameter(parameter->getName(),parameter->getValue(),parameter->getType());
      delete parameter;
      E=E->getNextElementSibling();
    }
    return true;
  }

  DOMElement* ParameterList::writeXMLFile(DOMNode *parent) const {
    DOMDocument *doc=parent->getOwnerDocument();
    for(int i=0; i<getSize(); i++) {
      DOMElement *p=D(doc)->createElement(PARAM%type[i]);
      parent->insertBefore(p, NULL);
      if(type[i]=="searchPath") {
        E(p)->setAttribute("href", value[i]);
      }
      else {
      E(p)->setAttribute("name", name[i]);
      DOMText *t=doc->createTextNode(X()%value[i]);
      p->insertBefore(t, NULL);
      }
    }
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
    for(int i=0; i<parameter.size(); i++)
      delete parameter[i];
  }

  Parameters Parameters::readXMLFile(const string &filename) {
    MBSimObjectFactory::initialize();
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
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
    DOMElement *ele0 = D(doc)->createElement(PARAM%string("Parameter"));
    parent->insertBefore(ele0, NULL);
    for(int i=0; i<parameter.size(); i++)
      parameter[i]->writeXMLFile(ele0);
    return ele0;
  }

  void Parameters::writeXMLFile(const string &name) {
    shared_ptr<DOMDocument> doc=MainWindow::parser->createDocument();
    writeXMLFile(doc.get());
    QFileInfo info(QString::fromStdString(name));
    QDir dir;
    if(!dir.exists(info.absolutePath()))
      dir.mkpath(info.absolutePath());
    DOMParser::serialize(doc.get(), (name.length()>4 && name.substr(name.length()-4,4)==".xml")?name:name+".xml");
  }

}
