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
#include "rigidbody.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "basic_properties.h"
#include "kinematics_properties.h"
#include "ombv_properties.h"
#include "mainwindow.h"
#include <QMenu>

using namespace std;

extern MainWindow *mw;

RigidBody::RigidBody(const string &str, Element *parent) : Body(str,parent), constrained(false), K(0,false), translation(0,false), rotation(0,false), ombvEditor(0,true), weightArrow(0,false), jointForceArrow(0,false), jointMomentArrow(0,false), isFrameOfBodyForRotation(0,false) {
  Frame *C = new Frame("C",this);
  addFrame(C);

  K.setProperty(new LocalFrameOfReferenceProperty(getFrame(0),this,MBSIMNS"frameForKinematics"));

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"kg",MBSIMNS"mass"));
  mass.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new MatProperty(getEye<string>(3,3,"0.01","0")),"kg*m^2",MBSIMNS"inertiaTensor"));
  inertia.setProperty(new ExtPhysicalVarProperty(input));

  translation.setProperty(new TranslationChoiceProperty(new LinearTranslationProperty,""));
  translation.setXMLName(MBSIMNS"translation");

  rotation.setProperty(new RotationChoiceProperty(new RotationAboutZAxisProperty,""));
  rotation.setXMLName(MBSIMNS"rotation");

  ombvEditor.setProperty(new OMBVBodySelectionProperty(this));

  weightArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  weightArrow.setXMLName(MBSIMNS"openMBVWeightArrow",false);
  ((OMBVArrowProperty*)weightArrow.getProperty())->setID(getID());

  jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  jointForceArrow.setXMLName(MBSIMNS"openMBVJointForceArrow",false);
  ((OMBVArrowProperty*)jointForceArrow.getProperty())->setID(getID());

  jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  jointMomentArrow.setXMLName(MBSIMNS"openMBVJointMomentArrow",false);
  ((OMBVArrowProperty*)jointMomentArrow.getProperty())->setID(getID());

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMNS"isFrameOfBodyForRotation"));
  isFrameOfBodyForRotation.setProperty(new ExtPhysicalVarProperty(input)); 
}

RigidBody::~RigidBody() {
}

int RigidBody::getqRelSize() const {
  return (translation.isActive()?((TranslationChoiceProperty*)translation.getProperty())->getSize():0) + (rotation.isActive()?((RotationChoiceProperty*)rotation.getProperty())->getSize():0);
}

void RigidBody::initialize() {
  Body::initialize();

  for(int i=0; i<frame.size(); i++)
    frame[i]->initialize();
  for(int i=0; i<contour.size(); i++)
    contour[i]->initialize();
}

void RigidBody::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Body::initializeUsingXML(element);

  // frames
  e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
  while(e && e->ValueStr()==MBSIMNS"frame") {
    TiXmlElement *ec=e->FirstChildElement();
    FixedRelativeFrame *f=new FixedRelativeFrame(ec->Attribute("name"),this);
    addFrame(f);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(e);
    e=e->NextSiblingElement();
  }
  while(e && e->ValueStr()==MBSIMNS"FixedRelativeFrame") {
    FixedRelativeFrame *f=new FixedRelativeFrame(e->Attribute("name"),this);
    addFrame(f);
    f->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }

  // contours
  e=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  Contour *c;
  while(e && e->ValueStr()==MBSIMNS"contour") {
    TiXmlElement *ec=e->FirstChildElement();
    c=ObjectFactory::getInstance()->createContour(ec,this);
    if(c) {
      addContour(c);
      c->initializeUsingXML(ec);
    }
    stringstream stream;
    stream << "ContourFrame" << contour.size();
    FixedRelativeFrame *f=new FixedRelativeFrame(stream.str(),this);
    addFrame(f);
    f->initializeUsingXML2(e);
    c->setSavedFrameOfReference(string("../Frame[")+f->getName()+"]");
    e=e->NextSiblingElement();
  }
  while(e) {
    c=ObjectFactory::getInstance()->createContour(e,this);
    if(c) {
      addContour(c);
      c->initializeUsingXML(e);
    }
    e=e->NextSiblingElement();
  }

  K.initializeUsingXML(element);

  mass.initializeUsingXML(element);
  inertia.initializeUsingXML(element);

  translation.initializeUsingXML(element);
  rotation.initializeUsingXML(element);

  isFrameOfBodyForRotation.initializeUsingXML(element);

  ombvEditor.initializeUsingXML(element);

  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameC");
  if(e)
    getFrame(0)->initializeUsingXML2(e);
  else
    getFrame(0)->setOpenMBVFrame(false);

  weightArrow.initializeUsingXML(element);

  jointForceArrow.initializeUsingXML(element);
  jointMomentArrow.initializeUsingXML(element);

  Body::initializeUsingXML(element);
}

TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Body::writeXMLFile(parent);
  TiXmlElement *ele1;

  K.writeXMLFile(ele0);

  mass.writeXMLFile(ele0);
  inertia.writeXMLFile(ele0);

  translation.writeXMLFile(ele0);
  rotation.writeXMLFile(ele0);

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  for(int i=1; i<frame.size(); i++)
    frame[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  for(int i=0; i<contour.size(); i++)
    contour[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  isFrameOfBodyForRotation.writeXMLFile(ele0);

  ombvEditor.writeXMLFile(ele0);

  Frame *C = getFrame(0);
  if(C->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
    C->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
  }

  weightArrow.writeXMLFile(ele0);

  jointForceArrow.writeXMLFile(ele0);
  jointMomentArrow.writeXMLFile(ele0);

  return ele0;
}
