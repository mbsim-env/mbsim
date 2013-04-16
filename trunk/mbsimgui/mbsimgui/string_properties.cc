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
#include "string_properties.h"
#include "frame.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QDir>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;

extern QDir mbsDir;
extern bool absolutePath;

void StringProperty::fromWidget(QWidget *widget) {
  setValue(static_cast<StringWidget*>(widget)->getValue());
}

void StringProperty::toWidget(QWidget *widget) {
  static_cast<StringWidget*>(widget)->setValue(getValue());
}

//TiXmlElement* OctaveExpressionProperty::initializeUsingXML(TiXmlElement *element) {
//  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
//  if(!text)
//    return 0;
//  setValue(text->Value());
//  return element;
//}
//
//TiXmlElement* OctaveExpressionProperty::writeXMLFile(TiXmlNode *parent) {
//  TiXmlText *text = new TiXmlText(getValue());
//  parent->LinkEndChild(text);
//  return 0;
//}

TiXmlElement* ScalarProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  setValue(text->Value());
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
  int i=0;
  value.clear();
  while(ei && ei->ValueStr()==PVNS"ele") {
    value.push_back(ei->GetText());
    ei=ei->NextSiblingElement();
    i++;
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
  setVec(static_cast<BasicVecWidget*>(widget)->getVec());
}

void VecProperty::toWidget(QWidget *widget) {
  static_cast<BasicVecWidget*>(widget)->setVec(getVec());
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
  setMat(static_cast<BasicMatWidget*>(widget)->getMat());
}

void MatProperty::toWidget(QWidget *widget) {
  static_cast<BasicMatWidget*>(widget)->setMat(getMat());
}

//SymMatProperty::SymMatProperty(int rows) {
// value.resize(rows);
//  for(int i=0; i<rows; i++)
//    value[i].resize(rows);
//}
//
//TiXmlElement* SymMatProperty::initializeUsingXML(TiXmlElement *parent) {
//  TiXmlElement *element=parent->FirstChildElement();
//  if(!element || element->ValueStr() != (PVNS"xmlMatrix"))
//    return 0;
//  TiXmlElement *ei=element->FirstChildElement();
//  int i=0;
//  while(ei && ei->ValueStr()==PVNS"row") {
//    TiXmlElement *ej=ei->FirstChildElement();
//    int j=0;
//    value.push_back(vector<string>());
//    while(ej && ej->ValueStr()==PVNS"ele") {
//      value[i].push_back(ej->GetText());
//      ej=ej->NextSiblingElement();
//      j++;
//    }
//    i++;
//    ei=ei->NextSiblingElement();
//  }
//  return element;
//}
//
//TiXmlElement* SymMatProperty::writeXMLFile(TiXmlNode *parent) {
//  TiXmlElement *ele = new TiXmlElement(PVNS"xmlMatrix");
//  for(unsigned int i=0; i<rows(); i++) {
//    TiXmlElement *elei = new TiXmlElement(PVNS"row");
//    for(unsigned int j=0; j<cols(); j++) {
//      TiXmlElement *elej = new TiXmlElement(PVNS"ele");
//      TiXmlText *text = new TiXmlText(value[i][j]);
//      elej->LinkEndChild(text);
//      elei->LinkEndChild(elej);
//    }
//    ele->LinkEndChild(elei);
//  }
//  parent->LinkEndChild(ele);
//  return 0;
//}
//
//void SymMatProperty::fromWidget(QWidget *widget) {
//  setMat(static_cast<BasicMatWidget*>(widget)->getMat());
//}
//
//void SymMatProperty::toWidget(QWidget *widget) {
//  static_cast<BasicMatWidget*>(widget)->setMat(getMat());
//}

TiXmlElement* PhysicalStringProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    if(value->initializeUsingXML(e)) {
      if(e->Attribute("unit"))
        setUnit(e->Attribute("unit"));
      return e;
    }
  } 
  return 0;
}

TiXmlElement* PhysicalStringProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  if(unit!="")
    ele->SetAttribute("unit", getUnit());
  value->writeXMLFile(ele);
  parent->LinkEndChild(ele);
  return 0;
}

void PhysicalStringProperty::fromWidget(QWidget *widget) {
  getProperty()->fromWidget(static_cast<PhysicalStringWidget*>(widget)->getWidget());
  setUnit(static_cast<PhysicalStringWidget*>(widget)->getUnit());
}

void PhysicalStringProperty::toWidget(QWidget *widget) {
  getProperty()->toWidget(static_cast<PhysicalStringWidget*>(widget)->getWidget());
  static_cast<PhysicalStringWidget*>(widget)->setUnit(getUnit());
}

string VecFromFileProperty::getValue() const {
  return evalOctaveExpression(string("load('") + fileName.toStdString() + "')");
}

TiXmlElement* VecFromFileProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  string str = text->Value();
  if(str.substr(0,4)!="load")
    return 0;
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  fileName = str.substr(pos1+1,pos2-pos1-1).c_str();
  absoluteFilePath=mbsDir.absoluteFilePath(str.substr(pos1+1,pos2-pos1-1).c_str());

  return element;
}

TiXmlElement* VecFromFileProperty::writeXMLFile(TiXmlNode *parent) {
  QString filePath = QString("load('")+(absolutePath?absoluteFilePath:mbsDir.relativeFilePath(absoluteFilePath))+"')";
 //string exp = string("load('") + fileName.toStdString() + "')"; 
  TiXmlText *text = new TiXmlText(filePath.toStdString());
  parent->LinkEndChild(text);
  return 0;
}

void VecFromFileProperty::fromWidget(QWidget *widget) {
  fileName = static_cast<VecFromFileWidget*>(widget)->fileName->text();
  absoluteFilePath = static_cast<VecFromFileWidget*>(widget)->absoluteFilePath;
}

void VecFromFileProperty::toWidget(QWidget *widget) {
  static_cast<VecFromFileWidget*>(widget)->fileName->setText(fileName);
  static_cast<VecFromFileWidget*>(widget)->absoluteFilePath = absoluteFilePath;
}

string MatFromFileProperty::getValue() const {
  return evalOctaveExpression(string("load('") + fileName.toStdString() + "')");
}

TiXmlElement* MatFromFileProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlText* text = dynamic_cast<TiXmlText*>(element->FirstChild());
  if(!text)
    return 0;
  string str = text->Value();
  if(str.substr(0,4)!="load")
    return 0;
  int pos1 = str.find_first_of('\''); 
  int pos2 = str.find_last_of('\''); 
  fileName = str.substr(pos1+1,pos2-pos1-1).c_str();
  absoluteFilePath=mbsDir.absoluteFilePath(str.substr(pos1+1,pos2-pos1-1).c_str());
  return element;
}

TiXmlElement* MatFromFileProperty::writeXMLFile(TiXmlNode *parent) {
  QString filePath = QString("load('")+(absolutePath?absoluteFilePath:mbsDir.relativeFilePath(absoluteFilePath))+"')";
 //string exp = string("load('") + fileName->text().toStdString() + "')"; 
  TiXmlText *text = new TiXmlText(filePath.toStdString());
  parent->LinkEndChild(text);
  return 0;
}

void MatFromFileProperty::fromWidget(QWidget *widget) {
  fileName = static_cast<MatFromFileWidget*>(widget)->fileName->text();
  absoluteFilePath = static_cast<MatFromFileWidget*>(widget)->absoluteFilePath;
}

void MatFromFileProperty::toWidget(QWidget *widget) {
  static_cast<MatFromFileWidget*>(widget)->fileName->setText(fileName);
  static_cast<MatFromFileWidget*>(widget)->absoluteFilePath = absoluteFilePath;
}
