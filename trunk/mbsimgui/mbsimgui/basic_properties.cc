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
#include "basic_properties.h"
#include "frame.h"
#include "contour.h"
#include "rigidbody.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include <QDir>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;
using namespace MBXMLUtils;

extern bool absolutePath;
extern QDir mbsDir;

TiXmlElement* LocalFrameOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    string refF="";
    refF=e->Attribute("ref");
    refF=refF.substr(6, refF.length()-7);
    setFrame(refF==""?element->getFrame(0):element->getFrame(refF));
  }
  return e;
}

TiXmlElement* LocalFrameOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  string str = string("Frame[") + getFrame()->getName() + "]";
  ele->SetAttribute("ref", str);
  parent->LinkEndChild(ele);
  return 0;
}

void LocalFrameOfReferenceProperty::fromWidget(QWidget *widget) {
  setFrame(static_cast<LocalFrameOfReferenceWidget*>(widget)->getFrame());
}

void LocalFrameOfReferenceProperty::toWidget(QWidget *widget) {
  static_cast<LocalFrameOfReferenceWidget*>(widget)->setFrame(getFrame());
  static_cast<LocalFrameOfReferenceWidget*>(widget)->updateWidget();
}

void ParentFrameOfReferenceProperty::initialize() {
  if(saved_frameOfReference!="") {
    string refF = saved_frameOfReference;
    refF=refF.substr(9, refF.length()-10);
    setFrame(element->getParent()->getFrame(refF));
  }
}

TiXmlElement* ParentFrameOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) saved_frameOfReference = e->Attribute("ref");
  return e;
}

TiXmlElement* ParentFrameOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  string str = string("../Frame[") + getFrame()->getName() + "]";
  ele->SetAttribute("ref", str);
  parent->LinkEndChild(ele);
  return 0;
}

void ParentFrameOfReferenceProperty::fromWidget(QWidget *widget) {
  setFrame(static_cast<ParentFrameOfReferenceWidget*>(widget)->getFrame());
}

void ParentFrameOfReferenceProperty::toWidget(QWidget *widget) {
  static_cast<ParentFrameOfReferenceWidget*>(widget)->setFrame(getFrame());
  static_cast<ParentFrameOfReferenceWidget*>(widget)->updateWidget();
}

void FrameOfReferenceProperty::initialize() {
  if(saved_frameOfReference!="")
    setFrame(element->getByPath<Frame>(saved_frameOfReference));
}

TiXmlElement* FrameOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) saved_frameOfReference=e->Attribute("ref");
  return e;
}

TiXmlElement* FrameOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  if(getFrame()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getFrame()->getXMLPath(element,true));
    parent->LinkEndChild(ele);
  }
  return 0;
}

void FrameOfReferenceProperty::fromWidget(QWidget *widget) {
  setFrame(static_cast<FrameOfReferenceWidget*>(widget)->getFrame());
}

void FrameOfReferenceProperty::toWidget(QWidget *widget) {
  static_cast<FrameOfReferenceWidget*>(widget)->setFrame(getFrame());
  static_cast<FrameOfReferenceWidget*>(widget)->updateWidget();
}

void ContourOfReferenceProperty::initialize() {
  if(saved_contourOfReference!="")
    setContour(element->getByPath<Contour>(saved_contourOfReference));
}

TiXmlElement* ContourOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) saved_contourOfReference=e->Attribute("ref");
  return e;
}

TiXmlElement* ContourOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  if(getContour()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getContour()->getXMLPath(element,true));
    parent->LinkEndChild(ele);
  }
  return 0;
}

void ContourOfReferenceProperty::fromWidget(QWidget *widget) {
  setContour(static_cast<ContourOfReferenceWidget*>(widget)->getContour());
}

void ContourOfReferenceProperty::toWidget(QWidget *widget) {
  static_cast<ContourOfReferenceWidget*>(widget)->setContour(getContour());
  static_cast<ContourOfReferenceWidget*>(widget)->updateWidget();
}

void RigidBodyOfReferenceProperty::initialize() {
  if(saved_bodyOfReference!="")
    setBody(element->getByPath<RigidBody>(saved_bodyOfReference));
}

TiXmlElement* RigidBodyOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) saved_bodyOfReference=e->Attribute("ref");
  return e;
}

TiXmlElement* RigidBodyOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  if(getBody()) {
    TiXmlElement *ele = new TiXmlElement(xmlName);
    ele->SetAttribute("ref", getBody()->getXMLPath(element,true));
    parent->LinkEndChild(ele);
  }
  return 0;
}

void RigidBodyOfReferenceProperty::fromWidget(QWidget *widget) {
  setBody(static_cast<RigidBodyOfReferenceWidget*>(widget)->getBody());
}

void RigidBodyOfReferenceProperty::toWidget(QWidget *widget) {
  static_cast<RigidBodyOfReferenceWidget*>(widget)->setBody(getBody());
  static_cast<RigidBodyOfReferenceWidget*>(widget)->updateWidget();
}

string FileProperty::getAbsoluteFilePath() const {
  return absolutePath?absoluteFilePath:mbsDir.relativeFilePath(QString::fromStdString(absoluteFilePath)).toStdString();
}

void FileProperty::setAbsoluteFilePath(const string &str) {
  absoluteFilePath=mbsDir.absoluteFilePath(QString::fromStdString(str)).toStdString();
}

TiXmlElement* FileProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlText *text = dynamic_cast<TiXmlText*>(e->FirstChild());
    if(text) {
      string file = text->Value();
      file = file.substr(1,file.length()-2);
      fileName = file;
      setAbsoluteFilePath(file);
      return e;
    }
  }
  return 0;
}

TiXmlElement* FileProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  string filePath = string("\"")+getAbsoluteFilePath()+"\"";
  TiXmlText *text = new TiXmlText(filePath);
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);

  return 0;
}

void FileProperty::fromWidget(QWidget *widget) {
  fileName = static_cast<FileWidget*>(widget)->getFileName().toStdString();
  absoluteFilePath = static_cast<FileWidget*>(widget)->getAbsoluteFilePath().toStdString();
}

void FileProperty::toWidget(QWidget *widget) {
  static_cast<FileWidget*>(widget)->blockSignals(true);
  static_cast<FileWidget*>(widget)->setFileName(QString::fromStdString(fileName));
  static_cast<FileWidget*>(widget)->blockSignals(false);
  static_cast<FileWidget*>(widget)->setAbsoluteFilePath(QString::fromStdString(absoluteFilePath));
}

TiXmlElement* TextProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlText *text_ = dynamic_cast<TiXmlText*>(e->FirstChild());
    if(text_) {
      text = text_->Value();
      if(quote)
        text = text.substr(1,text.length()-2);
     return e;
    }
  }
  return 0;
}

TiXmlElement* TextProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  TiXmlText *text_ = new TiXmlText(quote?("\""+text+"\""):text);

  ele0->LinkEndChild(text_);
  parent->LinkEndChild(ele0);

  return 0;
}

void TextProperty::fromWidget(QWidget *widget) {
  text = static_cast<BasicTextWidget*>(widget)->getText().toStdString();
}

void TextProperty::toWidget(QWidget *widget) {
  static_cast<BasicTextWidget*>(widget)->setText(QString::fromStdString(text));
}

void DependenciesProperty::initialize() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->initialize();
}

void DependenciesProperty::addDependency() {
}

TiXmlElement* DependenciesProperty::initializeUsingXML(TiXmlElement *ele) {
  TiXmlElement *e = ele->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee=e->FirstChildElement();
    while(ee) {
      refBody.push_back(new RigidBodyOfReferenceProperty(0,element,MBSIMNS"dependentRigidBody"));
      refBody[refBody.size()-1]->setSavedBodyOfReference(ee->Attribute("ref"));
      ee=ee->NextSiblingElement();
    }
  }
  return e;
}

TiXmlElement* DependenciesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  for(int i=0; i<refBody.size(); i++) {
    if(refBody[i])
      refBody[i]->writeXMLFile(ele);
  }
  parent->LinkEndChild(ele);
  return ele;
}

void DependenciesProperty::fromWidget(QWidget *widget) {
  if(refBody.size()!=static_cast<DependenciesWidget*>(widget)->refBody.size()) {
    refBody.clear();
    for(int i=0; i<static_cast<DependenciesWidget*>(widget)->refBody.size(); i++)
      refBody.push_back(new RigidBodyOfReferenceProperty(0,element,MBSIMNS"dependentRigidBody"));
  }
  for(int i=0; i<static_cast<DependenciesWidget*>(widget)->refBody.size(); i++) {
    if(static_cast<DependenciesWidget*>(widget)->refBody[i])
      refBody[i]->fromWidget(static_cast<DependenciesWidget*>(widget)->refBody[i]);
  }
}

void DependenciesProperty::toWidget(QWidget *widget) {
  static_cast<DependenciesWidget*>(widget)->setNumberOfBodies(refBody.size());
  for(int i=0; i<refBody.size(); i++) {
    if(refBody[i]) {
      refBody[i]->toWidget(static_cast<DependenciesWidget*>(widget)->refBody[i]);
    }
  }
  static_cast<DependenciesWidget*>(widget)->updateWidget();
}

ConnectFramesProperty::ConnectFramesProperty(int n, Element *element_) : element(element_) {

  for(int i=0; i<n; i++) {
    string xmlName = MBSIMNS"ref";
    if(n>1)
      xmlName += toStr(i+1);
    frame.push_back(new FrameOfReferenceProperty(0,element,xmlName));
  }
}

void ConnectFramesProperty::initialize() {
  for(unsigned int i=0; i<frame.size(); i++)
    frame[i]->initialize();
}

TiXmlElement* ConnectFramesProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"connect");
  if(e) {
    for(unsigned int i=0; i<frame.size(); i++) {
      string xmlName = "ref";
      if(frame.size()>1)
        xmlName += toStr(int(i+1));
      if(!e->Attribute(xmlName))
        return 0;
      frame[i]->setSavedFrameOfReference(e->Attribute(xmlName.c_str()));
    }
  }
  return e;
}

TiXmlElement* ConnectFramesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(MBSIMNS"connect");
  for(unsigned int i=0; i<frame.size(); i++) {
    string xmlName = "ref";
    if(frame.size()>1)
      xmlName += toStr(int(i+1));
    if(frame[i]->getFrame())
      ele->SetAttribute(xmlName, frame[i]->getFrame()->getXMLPath(element,true)); 
  }
  parent->LinkEndChild(ele);
  return ele;
}

void ConnectFramesProperty::fromWidget(QWidget *widget) {
  for(unsigned int i=0; i<frame.size(); i++)
    frame[i]->fromWidget(static_cast<ConnectFramesWidget*>(widget)->widget[i]);
}

void ConnectFramesProperty::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<frame.size(); i++)
    frame[i]->toWidget(static_cast<ConnectFramesWidget*>(widget)->widget[i]);
  static_cast<ConnectFramesWidget*>(widget)->update();
}

ConnectContoursProperty::ConnectContoursProperty(int n, Element *element_) : element(element_) {

  for(int i=0; i<n; i++) {
    string xmlName = MBSIMNS"ref";
    if(n>1)
      xmlName += toStr(i+1);
    contour.push_back(new ContourOfReferenceProperty(0,element,xmlName));
  }
}

void ConnectContoursProperty::initialize() {
  for(unsigned int i=0; i<contour.size(); i++)
    contour[i]->initialize();
}

TiXmlElement* ConnectContoursProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"connect");
  if(e) {
    for(unsigned int i=0; i<contour.size(); i++) {
      string xmlName = "ref";
      if(contour.size()>1)
        xmlName += toStr(int(i+1));
      if(!e->Attribute(xmlName))
        return 0;
      contour[i]->setSavedContourOfReference(e->Attribute(xmlName.c_str()));
    }
  }
  return e;
}

TiXmlElement* ConnectContoursProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(MBSIMNS"connect");
  for(unsigned int i=0; i<contour.size(); i++) {
    string xmlName = "ref";
    if(contour.size()>1)
      xmlName += toStr(int(i+1));
    if(contour[i]->getContour())
      ele->SetAttribute(xmlName, contour[i]->getContour()->getXMLPath(element,true)); 
  }
  parent->LinkEndChild(ele);
  return ele;
}

void ConnectContoursProperty::fromWidget(QWidget *widget) {
  for(unsigned int i=0; i<contour.size(); i++)
    contour[i]->fromWidget(static_cast<ConnectContoursWidget*>(widget)->widget[i]);
}

void ConnectContoursProperty::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<contour.size(); i++)
    contour[i]->toWidget(static_cast<ConnectContoursWidget*>(widget)->widget[i]);
  static_cast<ConnectContoursWidget*>(widget)->update();
}

SolverTolerancesProperty::SolverTolerancesProperty() : g(0,false), gd(0,false), gdd(0,false), la(0,false), La(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-15"), "-", MBSIMNS"projection"));
  projection.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-8"), "-", MBSIMNS"g"));
  g.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-10"), "-", MBSIMNS"gd"));
  gd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-12"), "-", MBSIMNS"gdd"));
  gdd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-12"), "-", MBSIMNS"la"));
  la.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1e-10"), "-", MBSIMNS"La"));
  La.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SolverTolerancesProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"tolerances");
  if(e) {
    projection.initializeUsingXML(e);
    g.initializeUsingXML(e);
    gd.initializeUsingXML(e);
    gdd.initializeUsingXML(e);
    la.initializeUsingXML(e);
    La.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* SolverTolerancesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(MBSIMNS"tolerances");
  parent->LinkEndChild(e);
  projection.writeXMLFile(e);
  g.writeXMLFile(e);
  gd.writeXMLFile(e);
  gdd.writeXMLFile(e);
  la.writeXMLFile(e);
  La.writeXMLFile(e);
  return e;
}

void SolverTolerancesProperty::fromWidget(QWidget *widget) {
  projection.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->projection);
  g.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->g);
  gd.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->gd);
  gdd.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->gdd);
  la.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->la);
  La.fromWidget(static_cast<SolverTolerancesWidget*>(widget)->La);
}

void SolverTolerancesProperty::toWidget(QWidget *widget) {
  projection.toWidget(static_cast<SolverTolerancesWidget*>(widget)->projection);
  g.toWidget(static_cast<SolverTolerancesWidget*>(widget)->g);
  gd.toWidget(static_cast<SolverTolerancesWidget*>(widget)->gd);
  gdd.toWidget(static_cast<SolverTolerancesWidget*>(widget)->gdd);
  la.toWidget(static_cast<SolverTolerancesWidget*>(widget)->la);
  La.toWidget(static_cast<SolverTolerancesWidget*>(widget)->La);
}

SolverParametersProperty::SolverParametersProperty() : tolerances(0,false) {
  tolerances.setProperty(new SolverTolerancesProperty);
}

TiXmlElement* SolverParametersProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"solverParameters");
  if(e) tolerances.initializeUsingXML(e);
  return e;
}

TiXmlElement* SolverParametersProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(MBSIMNS"solverParameters");
  parent->LinkEndChild(e);
  tolerances.writeXMLFile(e);
  return e;
}

void SolverParametersProperty::fromWidget(QWidget *widget) {
  tolerances.fromWidget(static_cast<SolverParametersWidget*>(widget)->tolerances);
}

void SolverParametersProperty::toWidget(QWidget *widget) {
  tolerances.toWidget(static_cast<SolverParametersWidget*>(widget)->tolerances);
}

GearDependencyProperty::GearDependencyProperty(Element* element_) : element(element_), refBody(0,element) {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "", MBSIMNS"transmissionRatio"));
  ratio.setProperty(new ExtPhysicalVarProperty(input));
} 

TiXmlElement* GearDependencyProperty::initializeUsingXML(TiXmlElement *ele) {
  ratio.initializeUsingXML(ele);
  refBody.setSavedBodyOfReference(ele->Attribute("ref"));
  return ele;
}

TiXmlElement* GearDependencyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(MBSIMNS"independentRigidBody");
  ratio.writeXMLFile(ele);
  if(refBody.getBody())
    ele->SetAttribute("ref", refBody.getBody()->getXMLPath(element,true));
  parent->LinkEndChild(ele);
  return ele;
}

void GearDependencyProperty::fromWidget(QWidget *widget) {
  ratio.fromWidget(static_cast<GearDependencyWidget*>(widget)->ratio);
  refBody.fromWidget(static_cast<GearDependencyWidget*>(widget)->refBody);
}

void GearDependencyProperty::toWidget(QWidget *widget) {
  ratio.toWidget(static_cast<GearDependencyWidget*>(widget)->ratio);
  refBody.toWidget(static_cast<GearDependencyWidget*>(widget)->refBody);
}

void GearDependenciesProperty::initialize() {
  for(unsigned int i=0; i<refBody.size(); i++)
    refBody[i]->initialize();
}

void GearDependenciesProperty::addDependency() {
}

TiXmlElement* GearDependenciesProperty::initializeUsingXML(TiXmlElement *ele) {
  TiXmlElement *e = ele->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee=e->FirstChildElement();
    while(ee) {
      refBody.push_back(new GearDependencyProperty(element));
      refBody[refBody.size()-1]->initializeUsingXML(ee);
      ee=ee->NextSiblingElement();
    }
  }
  return e;
}

TiXmlElement* GearDependenciesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  for(int i=0; i<refBody.size(); i++) {
    if(refBody[i])
      refBody[i]->writeXMLFile(ele);
  }
  parent->LinkEndChild(ele);
  return ele;
}

void GearDependenciesProperty::fromWidget(QWidget *widget) {
  if(refBody.size()!=static_cast<GearDependenciesWidget*>(widget)->refBody.size()) {
    refBody.clear();
    for(int i=0; i<static_cast<GearDependenciesWidget*>(widget)->refBody.size(); i++)
      refBody.push_back(new GearDependencyProperty(element));
  }
  for(int i=0; i<static_cast<GearDependenciesWidget*>(widget)->refBody.size(); i++) {
    if(static_cast<GearDependenciesWidget*>(widget)->refBody[i])
      refBody[i]->fromWidget(static_cast<GearDependenciesWidget*>(widget)->refBody[i]);
  }
}

void GearDependenciesProperty::toWidget(QWidget *widget) {
  static_cast<GearDependenciesWidget*>(widget)->setNumberOfBodies(refBody.size());
  for(int i=0; i<refBody.size(); i++) {
    if(refBody[i]) {
      refBody[i]->toWidget(static_cast<GearDependenciesWidget*>(widget)->refBody[i]);
    }
  }
  static_cast<GearDependenciesWidget*>(widget)->updateWidget();
}

EmbedProperty::EmbedProperty(Element *element) : count(0,false), counterName(0,false), parameterList(0,false) {
  href.setProperty(new FileProperty(""));
  static_cast<FileProperty*>(href.getProperty())->setFileName(element->getName()+".xml");
  static_cast<FileProperty*>(href.getProperty())->setAbsoluteFilePath(element->getName()+".xml");
  count.setProperty(new TextProperty("1",""));
  counterName.setProperty(new TextProperty("n",""));
  parameterList.setProperty(new FileProperty(""));
}

TiXmlElement* EmbedProperty::initializeUsingXML(TiXmlElement *parent) {
  string file = parent->Attribute("href");
  static_cast<FileProperty*>(href.getProperty())->setFileName(file);
  static_cast<FileProperty*>(href.getProperty())->setAbsoluteFilePath(file);
  if(parent->Attribute("count")) {
    count.setActive(true);
    static_cast<TextProperty*>(count.getProperty())->setText(parent->Attribute("count"));
  }
  if(parent->Attribute("counterName")) {
    counterName.setActive(true);
    static_cast<TextProperty*>(counterName.getProperty())->setText(parent->Attribute("counterName"));
  }
  TiXmlElement *ele = parent->FirstChildElement(PVNS+string("localParameter"));
  if(ele) {
    parameterList.setActive(true);
    string file = ele->Attribute("href");
    static_cast<FileProperty*>(parameterList.getProperty())->setFileName(file);
    static_cast<FileProperty*>(parameterList.getProperty())->setAbsoluteFilePath(file);
  }
}

TiXmlElement* EmbedProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(PVNS+string("embed"));
  ele0->SetAttribute("href", static_cast<FileProperty*>(href.getProperty())->getFileName());
  if(count.isActive())
    ele0->SetAttribute("count", static_cast<TextProperty*>(count.getProperty())->getText());
  if(counterName.isActive())
    ele0->SetAttribute("counterName", static_cast<TextProperty*>(counterName.getProperty())->getText());
  if(parameterList.isActive()) {
    TiXmlElement *ele1=new TiXmlElement(PVNS+string("localParameter"));
    ele1->SetAttribute("href", static_cast<FileProperty*>(parameterList.getProperty())->getFileName());
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

void EmbedProperty::fromWidget(QWidget *widget) {
  href.fromWidget(static_cast<EmbedWidget*>(widget)->href);
  count.fromWidget(static_cast<EmbedWidget*>(widget)->count);
  counterName.fromWidget(static_cast<EmbedWidget*>(widget)->counterName);
  parameterList.fromWidget(static_cast<EmbedWidget*>(widget)->parameterList);
}

void EmbedProperty::toWidget(QWidget *widget) {
  href.toWidget(static_cast<EmbedWidget*>(widget)->href);
  count.toWidget(static_cast<EmbedWidget*>(widget)->count);
  counterName.toWidget(static_cast<EmbedWidget*>(widget)->counterName);
  parameterList.toWidget(static_cast<EmbedWidget*>(widget)->parameterList);
}

