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
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;
using namespace MBXMLUtils;

extern QDir mbsDir;
extern bool absolutePath;

vector<string> toStdVec(const vector<QString> &x) {
  vector<string> y(x.size());
  for(unsigned int i=0; i<x.size(); i++)
    y[i] = x[i].toStdString();
  return y;
}

vector<QString> fromStdVec(const vector<string> &x) {
  vector<QString> y(x.size());
  for(unsigned int i=0; i<x.size(); i++)
    y[i] = QString::fromStdString(x[i]);
  return y;
}

vector<vector<string> > toStdMat(const vector<vector<QString> > &A) {
  vector<vector<string> > B(A.size());
  for(unsigned int i=0; i<A.size(); i++) {
    B[i].resize(A[i].size());
    for(unsigned int j=0; j<A[i].size(); j++)
      B[i][j] = A[i][j].toStdString();
  }
  return B;
}

vector<vector<QString> > fromStdMat(const vector<vector<string> > &A) {
  vector<vector<QString> > B(A.size());
  for(unsigned int i=0; i<A.size(); i++) {
    B[i].resize(A[i].size());
    for(unsigned int j=0; j<A[i].size(); j++)
      B[i][j] = QString::fromStdString(A[i][j]);
  }
  return B;
}

void VariableProperty::fromWidget(QWidget *widget) {
  setValue(static_cast<VariableWidget*>(widget)->getValue().toStdString());
}

void VariableProperty::toWidget(QWidget *widget) {
  static_cast<VariableWidget*>(widget)->setValue(QString::fromStdString(getValue()));
}

TiXmlElement* OctaveExpressionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = element->FirstChildText();
  if(!text)
    return 0;
  setValue(text->Value());
  return element;
}

TiXmlElement* OctaveExpressionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(getValue());
  parent->LinkEndChild(text);
  return 0;
}

TiXmlElement* ScalarProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = element->FirstChildText();
  if(!text)
    return 0;
  string str = text->Value();
  if(str.find("\n")!=string::npos)
    return 0;
  setValue(str);
  return element;
}

TiXmlElement* ScalarProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlText *text = new TiXmlText(getValue());
  parent->LinkEndChild(text);
  return 0;
}

VecProperty::VecProperty(int size) : value(size) {
  for(int i=0; i<size; i++)
    value[i] = "0";
}

VecProperty::~VecProperty() {
}

TiXmlElement* VecProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlVector"))
    return 0;
  TiXmlElement *ei=element->FirstChildElement();
  value.clear();
  while(ei && ei->ValueStr()==PVNS"ele") {
    value.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
  }
  return element;
}

TiXmlElement* VecProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlVector");
  for(unsigned int i=0; i<size(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"ele");
    TiXmlText *text = new TiXmlText(value[i]);
    elei->LinkEndChild(text);
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
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
}

TiXmlElement* MatProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
    return 0;
  TiXmlElement *ei=element->FirstChildElement();
  int i=0;
  value.clear();
  while(ei && ei->ValueStr()==PVNS"row") {
    TiXmlElement *ej=ei->FirstChildElement();
    int j=0;
    value.push_back(vector<string>());
    while(ej && ej->ValueStr()==PVNS"ele") {
      value[i].push_back(ej->GetText());
      ej=ej->NextSiblingElement();
      j++;
    }
    i++;
    ei=ei->NextSiblingElement();
  }
  return element;
}

TiXmlElement* MatProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"xmlMatrix");
  for(unsigned int i=0; i<rows(); i++) {
    TiXmlElement *elei = new TiXmlElement(PVNS"row");
    for(unsigned int j=0; j<cols(); j++) {
      TiXmlElement *elej = new TiXmlElement(PVNS"ele");
      TiXmlText *text = new TiXmlText(value[i][j]);
      elej->LinkEndChild(text);
      elei->LinkEndChild(elej);
    }
    ele->LinkEndChild(elei);
  }
  parent->LinkEndChild(ele);
  return 0;
}

void MatProperty::fromWidget(QWidget *widget) {
  setMat(toStdMat(static_cast<BasicMatWidget*>(widget)->getMat()));
}

void MatProperty::toWidget(QWidget *widget) {
  static_cast<BasicMatWidget*>(widget)->setMat(fromStdMat(getMat()));
}

CardanProperty::CardanProperty() : angles(3,"0") {
}

CardanProperty::~CardanProperty() {
}

TiXmlElement* CardanProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"cardan"))
    return 0;
  angles.clear();
  TiXmlElement *ei=element->FirstChildElement(PVNS"alpha");
  angles.push_back(ei->GetText());
  ei=ei->NextSiblingElement();
  angles.push_back(ei->GetText());
  ei=ei->NextSiblingElement();
  angles.push_back(ei->GetText());
  return element;
}

TiXmlElement* CardanProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"cardan");
  TiXmlElement *elei = new TiXmlElement(PVNS"alpha");
  TiXmlText *text = new TiXmlText(angles[0]);
  elei->LinkEndChild(text);
  ele->LinkEndChild(elei);
  elei = new TiXmlElement(PVNS"beta");
  text = new TiXmlText(angles[1]);
  elei->LinkEndChild(text);
  ele->LinkEndChild(elei);
  elei = new TiXmlElement(PVNS"gamma");
  text = new TiXmlText(angles[2]);
  elei->LinkEndChild(text);
  ele->LinkEndChild(elei);
  parent->LinkEndChild(ele);
  return 0;
}

void CardanProperty::fromWidget(QWidget *widget) {
  setAngles(toStdVec(static_cast<CardanWidget*>(widget)->getAngles()));
}

void CardanProperty::toWidget(QWidget *widget) {
  static_cast<CardanWidget*>(widget)->setAngles(fromStdVec(getAngles()));
}

TiXmlElement* PhysicalVariableProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = (xmlName=="")?parent:parent->FirstChildElement(xmlName);
  if(e) {
    if(value->initializeUsingXML(e)) {
      if(e->Attribute("unit"))
        setUnit(e->Attribute("unit"));
      return e;
    }
  } 
  return 0;
}

TiXmlElement* PhysicalVariableProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele;
  if(xmlName!="") {
    ele = new TiXmlElement(xmlName);
    parent->LinkEndChild(ele);
  } 
  else
    ele = (TiXmlElement*)parent;
  if(unit!="")
    ele->SetAttribute("unit", getUnit());
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

string FromFileProperty::getValue() const {
  return OctEval::cast<string>(MainWindow::octEval->stringToOctValue("'" + file + "'"));
}

TiXmlElement* FromFileProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *element=parent->FirstChildElement();
  if(!element || element->ValueStr() != (PVNS"fromFile"))
    return 0;

  string str = element->Attribute("href");
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  file = str.substr(pos1+1,pos2-pos1-1).c_str();
  QFileInfo fileInfo(QString::fromStdString(file));
  file = fileInfo.canonicalFilePath().toStdString();
  return element;
}

// TiXmlText* text = element->FirstChildText();
//  if(!text)
//    return 0;
//  string str = text->Value();
//  if(str.substr(0,8)!="ret=load")
//    return 0;
//  int pos1 = str.find_first_of('\''); 
//  int pos2 = str.find_last_of('\''); 
//  file = str.substr(pos1+1,pos2-pos1-1).c_str();
//  QFileInfo fileInfo(QString::fromStdString(file));
//  file = fileInfo.canonicalFilePath().toStdString();
//  return element;
//}


TiXmlElement* FromFileProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(PVNS"fromFile");
  string filePath = "'"+(absolutePath?mbsDir.absoluteFilePath(QString::fromStdString(file)).toStdString():mbsDir.relativeFilePath(QString::fromStdString(file)).toStdString())+"'";
  //string filePath = file;
  ele->SetAttribute("href",filePath);
  parent->LinkEndChild(ele);
  return 0;
}

void FromFileProperty::fromWidget(QWidget *widget) {
  file = static_cast<FromFileWidget*>(widget)->getFile().toStdString();
  cout << file << endl;
}

void FromFileProperty::toWidget(QWidget *widget) {
  static_cast<FromFileWidget*>(widget)->blockSignals(true);
  static_cast<FromFileWidget*>(widget)->setFile(QString::fromStdString(file));
  static_cast<FromFileWidget*>(widget)->blockSignals(false);
}

ScalarPropertyFactory::ScalarPropertyFactory(const string &value_, const string &xmlName_) : value(value_), name(2), unit(2,"m"), xmlName(xmlName_) {
}

ScalarPropertyFactory::ScalarPropertyFactory(const string &value_, const string &xmlName_, const vector<string> &unit_) : value(value_), name(2), xmlName(xmlName_), unit(unit_) {
}

Property* ScalarPropertyFactory::createProperty(int i) {
  if(i==0)
    return new PhysicalVariableProperty(new ScalarProperty(value), unit[0], xmlName);
  if(i==1)
    return new PhysicalVariableProperty(new OctaveExpressionProperty, unit[1], xmlName);
}

VecPropertyFactory::VecPropertyFactory(int m, const string &xmlName_) : x(getScalars<string>(m,"0")), name(3), unit(3,"m"), xmlName(xmlName_) {
}

VecPropertyFactory::VecPropertyFactory(int m, const string &xmlName_, const vector<string> &unit_) : x(getScalars<string>(m,"0")), name(3), xmlName(xmlName_), unit(unit_) {
}

VecPropertyFactory::VecPropertyFactory(const vector<string> &x_, const string &xmlName_, const vector<string> &unit_) : x(x_), name(3), xmlName(xmlName_), unit(unit_) {
}

Property* VecPropertyFactory::createProperty(int i) {
  if(i==0)
    return new PhysicalVariableProperty(new VecProperty(x), unit[0], xmlName);
  if(i==1)
    return new PhysicalVariableProperty(new FromFileProperty,unit[1],xmlName);
  if(i==2)
    return new PhysicalVariableProperty(new OctaveExpressionProperty, unit[2], xmlName);
}

RotMatPropertyFactory::RotMatPropertyFactory(const string &xmlName_) : name(3), unit(3,"-"), xmlName(xmlName_) {
  unit[1] = "";
}

RotMatPropertyFactory::RotMatPropertyFactory(const string &xmlName_, const vector<string> &unit_) : name(3), xmlName(xmlName_), unit(unit_) {
}

Property* RotMatPropertyFactory::createProperty(int i) {
  if(i==0)
    return new PhysicalVariableProperty(new MatProperty(getEye<string>(3,3,"1","0")),unit[0],xmlName);
  if(i==1)
    return new PhysicalVariableProperty(new CardanProperty,unit[1],xmlName);
  if(i==2)
    return new PhysicalVariableProperty(new OctaveExpressionProperty,unit[2],xmlName);
}

MatPropertyFactory::MatPropertyFactory(const string &xmlName_) : name(3), unit(3,"-"), xmlName(xmlName_) {
}

MatPropertyFactory::MatPropertyFactory(const vector<vector<string> > &A_, const string &xmlName_, const vector<string> &unit_) : A(A_), name(3), xmlName(xmlName_), unit(unit_) {
}

Property* MatPropertyFactory::createProperty(int i) {
  if(i==0)
    return new PhysicalVariableProperty(new MatProperty(A),unit[0],xmlName);
  if(i==1)
    return new PhysicalVariableProperty(new FromFileProperty,unit[1],xmlName);
  if(i==2)
    return new PhysicalVariableProperty(new OctaveExpressionProperty,unit[2],xmlName);
}
