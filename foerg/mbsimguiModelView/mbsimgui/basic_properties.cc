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
#include "rigidbody.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "extended_widgets.h"
#include <QDir>
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;

extern bool absolutePath;
extern QDir mbsDir;

TiXmlElement* LocalFrameOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) {
    QString refF="";
    refF=e->Attribute("ref");
    refF=refF.mid(6, refF.length()-7);
    setFrame(refF==""?element->getFrame(0):element->getFrame(refF));
  }
  return e;
}

TiXmlElement* LocalFrameOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  QString str = QString("Frame[") + getFrame()->getName() + "]";
  ele->SetAttribute("ref", str.toStdString());
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
    QString refF = saved_frameOfReference;
    refF=refF.mid(9, refF.length()-10);
    setFrame(element->getParentElement()->getFrame(refF));
  }
}

TiXmlElement* ParentFrameOfReferenceProperty::initializeUsingXML(TiXmlElement *parent) {
  TiXmlElement *e = parent->FirstChildElement(xmlName);
  if(e) saved_frameOfReference = e->Attribute("ref");
  return e;
}

TiXmlElement* ParentFrameOfReferenceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  QString str = QString("../Frame[") + getFrame()->getName() + "]";
  ele->SetAttribute("ref", str.toStdString());
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
    ele->SetAttribute("ref", getFrame()->getXMLPath(element,true).toStdString());
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
    ele->SetAttribute("ref", getBody()->getXMLPath(element,true).toStdString());
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

TiXmlElement* FileProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlText *text = dynamic_cast<TiXmlText*>(e->FirstChild());
    if(text) {
      QString file = text->Value();
      fileName = file;
      file = file.mid(1,file.length()-2);
      absoluteFilePath=mbsDir.absoluteFilePath(file);
      return e;
    }
  }
  return 0;
}

TiXmlElement* FileProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  QString filePath = QString("\"")+(absolutePath?absoluteFilePath:mbsDir.relativeFilePath(absoluteFilePath))+"\"";
  TiXmlText *text = new TiXmlText(filePath.toStdString());
  ele0->LinkEndChild(text);
  parent->LinkEndChild(ele0);

  return 0;
}

void FileProperty::fromWidget(QWidget *widget) {
  fileName = static_cast<FileWidget*>(widget)->getFileName();
  absoluteFilePath = static_cast<FileWidget*>(widget)->getAbsoluteFilePath();
}

void FileProperty::toWidget(QWidget *widget) {
  static_cast<FileWidget*>(widget)->setFileName(fileName);
  static_cast<FileWidget*>(widget)->setAbsoluteFilePath(absoluteFilePath);
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
    QString xmlName = MBSIMNS"ref";
    if(n>1)
      xmlName += QString::number(i+1);
    frame.push_back(new FrameOfReferenceProperty(0,element,xmlName.toStdString()));
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
      QString xmlName = "ref";
      if(frame.size()>1)
        xmlName += QString::number(i+1);
      if(!e->Attribute(xmlName.toStdString()))
        return 0;
      frame[i]->setSavedFrameOfReference(e->Attribute(xmlName.toAscii().data()));
    }
  }
  return e;
}

TiXmlElement* ConnectFramesProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(MBSIMNS"connect");
  for(unsigned int i=0; i<frame.size(); i++) {
    QString xmlName = "ref";
    if(frame.size()>1)
      xmlName += QString::number(i+1);
    if(frame[i]->getFrame())
      ele->SetAttribute(xmlName.toAscii().data(), frame[i]->getFrame()->getXMLPath(element,true).toStdString()); 
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

SolverTolerancesProperty::SolverTolerancesProperty() : g(0,false), gd(0,false), gdd(0,false), la(0,false), La(0,false) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-15"), "-", MBSIMNS"projection"));
  projection.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-8"), "-", MBSIMNS"g"));
  g.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-10"), "-", MBSIMNS"gd"));
  gd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-12"), "-", MBSIMNS"gdd"));
  gdd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-12"), "-", MBSIMNS"la"));
  la.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1e-10"), "-", MBSIMNS"La"));
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

