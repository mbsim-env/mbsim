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
#include "variable_properties.h"
#include "frame.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include <QDir>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

extern QDir mbsDir;
extern bool absolutePath;

////void PhysicalProperty::setValue(const string &str) {
////  Property::setValue(str);
////  setEvaluation(OctEval::cast<string>(MainWindow::octEval->stringToOctValue(getValue())));
////}
//
//void PhysicalProperty::fromWidget(QWidget *widget) {
//  setValue(static_cast<VariableWidget*>(widget)->getValue().toStdString());
//}
//
//void PhysicalProperty::toWidget(QWidget *widget) {
//  static_cast<VariableWidget*>(widget)->setValue(QString::fromStdString(getValue()));
//}

DOMElement* OctaveExpressionProperty::initializeUsingXML(DOMElement *element) {
  PhysicalProperty::initializeUsingXML(element);
  DOMText* text = E(element)->getFirstTextChild();
  if(!text)
    return 0;
  setValue(X()%text->getData());
  return element;
}

DOMElement* OctaveExpressionProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  PhysicalProperty::writeXMLFile(parent);
  DOMText *text = doc->createTextNode(X()%getValue());
  parent->insertBefore(text, NULL);
  return 0;
}

DOMElement* ScalarProperty::initializeUsingXML(DOMElement *element) {
  PhysicalProperty::initializeUsingXML(element);
  DOMText* text = E(element)->getFirstTextChild();
  if(!text)
    return 0;
  string str = X()%text->getData();
  if(str.find("\n")!=string::npos)
    return 0;
  setValue(str);
  return element;
}

DOMElement* ScalarProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  PhysicalProperty::writeXMLFile(parent);
  DOMText *text = doc->createTextNode(X()%getValue());
  parent->insertBefore(text, NULL);
  return 0;
}

VecProperty::VecProperty(int size, const Units &unit) : PhysicalProperty("","",unit), value(size) {
  for(int i=0; i<size; i++)
    value[i] = "0";
  setValue(toStr(value)); 
}

VecProperty::~VecProperty() {
}

DOMElement* VecProperty::initializeUsingXML(DOMElement *parent) {
  PhysicalProperty::initializeUsingXML(parent);
  DOMElement *element=parent->getFirstElementChild();
  if(!element || E(element)->getTagName() != PV%"xmlVector")
    return 0;
  DOMElement *ei=element->getFirstElementChild();
  value.clear();
  while(ei && E(ei)->getTagName()==PV%"ele") {
    value.push_back(X()%E(ei)->getFirstTextChild()->getData());
    ei=ei->getNextElementSibling();
  }
  setValue(toStr(value));
  return element;
}

DOMElement* VecProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  PhysicalProperty::writeXMLFile(parent);
  DOMElement *ele = D(doc)->createElement(PV%"xmlVector");
  for(unsigned int i=0; i<size(); i++) {
    DOMElement *elei = D(doc)->createElement(PV%"ele");
    DOMText *text = doc->createTextNode(X()%value[i]);
    elei->insertBefore(text, NULL);
    ele->insertBefore(elei, NULL);
  }
  parent->insertBefore(ele, NULL);
  return 0;
}

void VecProperty::fromWidget(QWidget *widget) {
  setVec(toStdVec(static_cast<BasicVecWidget*>(widget)->getVec()));
}

void VecProperty::toWidget(QWidget *widget) {
  static_cast<BasicVecWidget*>(widget)->setVec(fromStdVec(getVec()));
}

MatProperty::MatProperty(int rows, int cols) {
 value.resize(rows);
 for(int i=0; i<rows; i++) {
   value[i].resize(cols);
   for(int j=0; j<cols; j++)
     value[i][j] = "0";
  }
  setValue(toStr(value)); 
}

DOMElement* MatProperty::initializeUsingXML(DOMElement *parent) {
  DOMElement *element=parent->getFirstElementChild();
  if(!element || E(element)->getTagName() != (PV%"xmlMatrix"))
    return 0;
  DOMElement *ei=element->getFirstElementChild();
  int i=0;
  value.clear();
  while(ei && E(ei)->getTagName()==PV%"row") {
    DOMElement *ej=ei->getFirstElementChild();
    int j=0;
    value.push_back(vector<string>());
    while(ej && E(ej)->getTagName()==PV%"ele") {
      value[i].push_back(X()%E(ej)->getFirstTextChild()->getData());
      ej=ej->getNextElementSibling();
      j++;
    }
    i++;
    ei=ei->getNextElementSibling();
  }
  setValue(toStr(value));
  return element;
}

DOMElement* MatProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele = D(doc)->createElement(PV%"xmlMatrix");
  for(unsigned int i=0; i<rows(); i++) {
    DOMElement *elei = D(doc)->createElement(PV%"row");
    for(unsigned int j=0; j<cols(); j++) {
      DOMElement *elej = D(doc)->createElement(PV%"ele");
      DOMText *text = doc->createTextNode(X()%value[i][j]);
      elej->insertBefore(text, NULL);
      elei->insertBefore(elej, NULL);
    }
    ele->insertBefore(elei, NULL);
  }
  parent->insertBefore(ele, NULL);
  return 0;
}

void MatProperty::fromWidget(QWidget *widget) {
  setMat(toStdMat(static_cast<BasicMatWidget*>(widget)->getMat()));
}

void MatProperty::toWidget(QWidget *widget) {
  static_cast<BasicMatWidget*>(widget)->setMat(fromStdMat(getMat()));
}

CardanProperty::CardanProperty(const string &name) : PhysicalProperty(name,"[0;0;0]",AngleUnits()), angles(3,"0") {
}

CardanProperty::~CardanProperty() {
}

DOMElement* CardanProperty::initializeUsingXML(DOMElement *parent) {
  DOMElement *element=parent->getFirstElementChild();
  if(!element || E(element)->getTagName() != (PV%"cardan"))
    return 0;
  angles.clear();
  DOMElement *ei=E(element)->getFirstElementChildNamed(PV%"alpha");
  angles.push_back(X()%E(ei)->getFirstTextChild()->getData());
  ei=ei->getNextElementSibling();
  angles.push_back(X()%E(ei)->getFirstTextChild()->getData());
  ei=ei->getNextElementSibling();
  angles.push_back(X()%E(ei)->getFirstTextChild()->getData());
  if(E(element)->hasAttribute("unit"))
    unit = E(element)->getAttribute("unit");
  setValue(toStr(angles));
  return element;
}

DOMElement* CardanProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele = D(doc)->createElement(PV%"cardan");
  DOMElement *elei = D(doc)->createElement(PV%"alpha");
  DOMText *text = doc->createTextNode(X()%angles[0]);
  elei->insertBefore(text, NULL);
  ele->insertBefore(elei, NULL);
  elei = D(doc)->createElement(PV%"beta");
  text = doc->createTextNode(X()%angles[1]);
  elei->insertBefore(text, NULL);
  ele->insertBefore(elei, NULL);
  elei = D(doc)->createElement(PV%"gamma");
  text = doc->createTextNode(X()%angles[2]);
  elei->insertBefore(text, NULL);
  ele->insertBefore(elei, NULL);
  if(unit!="")
    E(ele)->setAttribute("unit", unit);
  parent->insertBefore(ele, NULL);
  return 0;
}

void CardanProperty::fromWidget(QWidget *widget) {
//  setAngles(toStdVec(static_cast<CardanWidget*>(widget)->getAngles()));
//  unit = static_cast<CardanWidget*>(widget)->getUnit().toStdString();
//  //evaluation = OctEval::cast<string>(MainWindow::octEval->stringToOctValue(getValue()));
}

void CardanProperty::toWidget(QWidget *widget) {
//  static_cast<CardanWidget*>(widget)->setAngles(fromStdVec(getAngles()));
//  static_cast<CardanWidget*>(widget)->setUnit(QString::fromStdString(unit));
}

DOMElement* PhysicalVariableProperty::initializeUsingXML(DOMElement *parent) {
  DOMElement *e = (xmlName==FQN())?parent:E(parent)->getFirstElementChildNamed(xmlName);
  if(e) {
    if(value->initializeUsingXML(e)) {
      if(E(e)->hasAttribute("unit"))
        setUnit(E(e)->getAttribute("unit"));
      return e;
    }
  } 
  return 0;
}

DOMElement* PhysicalVariableProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele;
  if(xmlName!=FQN()) {
    DOMDocument *doc=parent->getOwnerDocument();
    ele = D(doc)->createElement(xmlName);
    parent->insertBefore(ele, NULL);
  } 
  else
    ele = (DOMElement*)parent;
  if(unit!="")
    E(ele)->setAttribute("unit", getUnit());
  value->writeXMLFile(ele);
  return 0;
}

void PhysicalVariableProperty::fromWidget(QWidget *widget) {
  getProperty()->fromWidget(static_cast<PhysicalVariableWidget*>(widget)->getWidget());
  setUnit(static_cast<PhysicalVariableWidget*>(widget)->getUnit().toStdString());
}

void PhysicalVariableProperty::toWidget(QWidget *widget) {
  getProperty()->toWidget(static_cast<PhysicalVariableWidget*>(widget)->getWidget());
  static_cast<PhysicalVariableWidget*>(widget)->setUnit(QString::fromStdString(getUnit()));
}

//string FromFileProperty::getValue() const {
//  return OctEval::cast<string>(MainWindow::octEval->stringToOctValue("'" + file + "'"));
//}

DOMElement* FromFileProperty::initializeUsingXML(DOMElement *parent) {
  DOMElement *element=parent->getFirstElementChild();
  if(!element || E(element)->getTagName() != (PV%"fromFile"))
    return 0;

  string str = E(element)->getAttribute("href");
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  file = str.substr(pos1+1,pos2-pos1-1).c_str();
  QFileInfo fileInfo(QString::fromStdString(file));
  file = fileInfo.canonicalFilePath().toStdString();
  return element;
}

// DOMText* text = E(element)->getFirstTextChild();
//  if(!text)
//    return 0;
//  string str = X()%text->getData();
//  if(str.substr(0,8)!="ret=load")
//    return 0;
//  int pos1 = str.find_first_of('\''); 
//  int pos2 = str.find_last_of('\''); 
//  file = str.substr(pos1+1,pos2-pos1-1).c_str();
//  QFileInfo fileInfo(QString::fromStdString(file));
//  file = fileInfo.canonicalFilePath().toStdString();
//  return element;
//}


DOMElement* FromFileProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele = D(doc)->createElement(PV%"fromFile");
  string filePath = "'"+(absolutePath?mbsDir.absoluteFilePath(QString::fromStdString(file)).toStdString():mbsDir.relativeFilePath(QString::fromStdString(file)).toStdString())+"'";
  //string filePath = file;
  E(ele)->setAttribute("href",filePath);
  parent->insertBefore(ele, NULL);
  return 0;
}

void FromFileProperty::fromWidget(QWidget *widget) {
  file = static_cast<FromFileWidget*>(widget)->getFile().toStdString();
}

void FromFileProperty::toWidget(QWidget *widget) {
  static_cast<FromFileWidget*>(widget)->blockSignals(true);
  static_cast<FromFileWidget*>(widget)->setFile(QString::fromStdString(file));
  static_cast<FromFileWidget*>(widget)->blockSignals(false);
}

ScalarPropertyFactory::ScalarPropertyFactory(const string &value_, const Units &unit_) : value(value_), name(2), unit(unit_) {
}

Property* ScalarPropertyFactory::createProperty(int i) {
  if(i==0)
    return new ScalarProperty("",value,unit);
  if(i==1)
    return new OctaveExpressionProperty("","",unit);
}

VecPropertyFactory::VecPropertyFactory(int m, const Units &unit_) : x(getScalars<string>(m,"0")), name(3), unit(unit_) {
}

VecPropertyFactory::VecPropertyFactory(const vector<string> &x_, const Units &unit_) : x(x_), name(3), unit(unit_) {
}

Property* VecPropertyFactory::createProperty(int i) {
  if(i==0)
    return new VecProperty("",x,unit);
  if(i==1)
    return new FromFileProperty;
  if(i==2)
    return new OctaveExpressionProperty("","",unit);
}

MatPropertyFactory::MatPropertyFactory(const Units &unit_) : name(3), unit(unit_) {
}
 
MatPropertyFactory::MatPropertyFactory(const vector<vector<string> > &A_, const Units &unit_) : A(A_), name(3), unit(unit_) {
}
 
Property* MatPropertyFactory::createProperty(int i) {
  if(i==0)
    return new MatProperty(A,unit);
  if(i==1)
    return new FromFileProperty;
  if(i==2)
    return new OctaveExpressionProperty("","",unit);
}

RotMatPropertyFactory::RotMatPropertyFactory() : name(3) {
}

Property* RotMatPropertyFactory::createProperty(int i) {
  if(i==0)
    return new CardanProperty("");
  if(i==1)
    return new MatProperty(getEye<string>(3,3,"1","0"));
  if(i==2)
    return new OctaveExpressionProperty("","",NoUnitUnits());
}

DOMElement* VariableProperty::initializeUsingXML(DOMElement *parent) {
  index = -1;
  for(int i=0; i<property.size(); i++) {
    if(property[i]->initializeUsingXML(parent)) {
      index = i;
      break;
    }
  }
  if(index == -1) {
    cout << "Mist" << endl;
    throw;
  }
}

DOMElement* VariableProperty::writeXMLFile(DOMNode *parent) {
  return property[index]->writeXMLFile(parent);
}

Scalar_Property::Scalar_Property(const string &name_, const Units &unit) : VariableProperty(name_) {
  property.push_back(new ScalarProperty("","1",unit));
  property.push_back(new OctaveExpressionProperty("","1",unit));
  name.push_back("xmlScalar");
  name.push_back("plain");
}

Vec_Property::Vec_Property(const string &name_, const Units &unit) : VariableProperty(name_) {
  property.push_back(new VecProperty(3,unit));
  property.push_back(new OctaveExpressionProperty("","[0;0;0]",unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

Vec_Property::Vec_Property(const string &name_, const vector<string> &x, const Units &unit) : VariableProperty(name_) {
  property.push_back(new VecProperty("",x,unit));
  property.push_back(new OctaveExpressionProperty("",toStr(x),unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

Mat_Property::Mat_Property(const string &name_, const Units &unit) : VariableProperty(name_) {
  property.push_back(new MatProperty(3,1));
  property.push_back(new OctaveExpressionProperty("","[0;0;0]",unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

Mat_Property::Mat_Property(const string &name_, const vector<vector<string> > &A, const Units &unit) : VariableProperty(name_) {
  property.push_back(new MatProperty("",A,unit));
  property.push_back(new OctaveExpressionProperty("",toStr(A),unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

SymMat_Property::SymMat_Property(const string &name_, const Units &unit) : VariableProperty(name_) {
  property.push_back(new SymMatProperty(getEye<string>(3,3,"0.01","0"),unit));
  property.push_back(new OctaveExpressionProperty("","0.01*eye(3)",unit));
  name.push_back("xmlMatrix");
  name.push_back("plain");
}

RotMatProperty::RotMatProperty(const string &name_) : VariableProperty(name_) {
  property.push_back(new MatProperty(getEye<string>(3,3,"1","0")));
  property.push_back(new CardanProperty);
  property.push_back(new OctaveExpressionProperty("","eye(3)"));
  name.push_back("xmlMatrix");
  name.push_back("cardan");
  name.push_back("plain");
}

VarMat_Property::VarMat_Property(const string &name_, const Units &unit) : VariableProperty(name_) {
  property.push_back(new VarMatProperty(3,1));
  property[0]->setParent(this);
  property.push_back(new OctaveExpressionProperty("","[0;0;0]",unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

VarMat_Property::VarMat_Property(const string &name_, const vector<vector<string> > &A, const Units &unit) : VariableProperty(name_) {
  property.push_back(new VarMatProperty("",A,unit));
  property[0]->setParent(this);
  property.push_back(new OctaveExpressionProperty("",toStr(A),unit));
  name.push_back("xmlVector");
  name.push_back("plain");
}

