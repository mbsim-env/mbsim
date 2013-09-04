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
#include "function_properties.h"

using namespace std;
using namespace MBXMLUtils;

RigidBody::RigidBody(const string &str, Element *parent) : Body(str,parent), constrained(false), K(0,false), translation(0,false), rotation(0,false), rotationMapping(0,false), ombvEditor(0,true), weightArrow(0,false), jointForceArrow(0,false), jointMomentArrow(0,false), isFrameOfBodyForRotation(0,false) {
  Frame *C = new Frame("C",this);
  addFrame(C);

  K.setProperty(new LocalFrameOfReferenceProperty("Frame[C]",this,MBSIMNS"frameForKinematics"));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"kg",MBSIMNS"mass"));
  mass.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatProperty(getEye<string>(3,3,"0.01","0")),"kg*m^2",MBSIMNS"inertiaTensor"));
  inertia.setProperty(new ExtPhysicalVarProperty(input));

  translation.setXMLName(MBSIMNS"translation");
  vector<Property*> property;
  property.push_back(new TranslationAlongXAxisProperty("V"));
  property.push_back(new TranslationAlongYAxisProperty("V"));
  property.push_back(new TranslationAlongZAxisProperty("V"));
  property.push_back(new TranslationAlongAxesXYProperty("V"));
  property.push_back(new TranslationAlongAxesYZProperty("V"));
  property.push_back(new TranslationAlongAxesXZProperty("V"));
  property.push_back(new TranslationAlongAxesXYZProperty("V"));
  property.push_back(new LinearFunctionProperty("VV",3,1));

  vector<Property*> property_;
  property_.push_back(new TranslationAlongXAxisProperty("S"));
  property_.push_back(new TranslationAlongYAxisProperty("S"));
  property_.push_back(new TranslationAlongZAxisProperty("S"));
  property.push_back(new NestedFunctionProperty("VSV",property_));

  property_.clear();
  property_.push_back(new TranslationAlongXAxisProperty("S"));
  property_.push_back(new TranslationAlongYAxisProperty("S"));
  property_.push_back(new TranslationAlongZAxisProperty("S"));
  property.push_back(new NestedFunctionProperty("VSS",property_));

  vector<string> var;
  var.push_back("q");
  property.push_back(new SymbolicFunctionProperty("VV",var));
  property.push_back(new ConstantFunctionProperty("VS",3));
  property.push_back(new LinearFunctionProperty("VS",3,1));
  property.push_back(new QuadraticFunctionProperty("V",3));
  property.push_back(new SinusFunctionProperty("V",3));
  var.clear();
  var.push_back("t");
  property.push_back(new SymbolicFunctionProperty("VS",var));
  var.clear();
  var.push_back("q");
  var.push_back("t");
  property.push_back(new SymbolicFunctionProperty("VVS",var));
  translation.setProperty(new ChoiceProperty("",property));

  rotation.setXMLName(MBSIMNS"rotation");
  property.clear();
  property.push_back(new RotationAboutXAxisProperty("V"));
  property.push_back(new RotationAboutYAxisProperty("V"));
  property.push_back(new RotationAboutZAxisProperty("V"));
  property.push_back(new RotationAboutAxesXYProperty("V"));
  property.push_back(new RotationAboutAxesYZProperty("V"));
  property.push_back(new RotationAboutAxesXZProperty("V"));
  property.push_back(new RotationAboutAxesXYZProperty("V"));
  property.push_back(new RotationAboutFixedAxisProperty("V"));

  property_.clear();
  property_.push_back(new RotationAboutXAxisProperty("S"));
  property_.push_back(new RotationAboutYAxisProperty("S"));
  property_.push_back(new RotationAboutZAxisProperty("S"));
  property_.push_back(new RotationAboutFixedAxisProperty("S"));
  property.push_back(new NestedFunctionProperty("MSV",property_));

  property.push_back(new RotationAboutFixedAxisProperty("S"));

  property_.clear();
  property_.push_back(new RotationAboutXAxisProperty("S"));
  property_.push_back(new RotationAboutYAxisProperty("S"));
  property_.push_back(new RotationAboutZAxisProperty("S"));
  property_.push_back(new RotationAboutFixedAxisProperty("S"));
  property.push_back(new NestedFunctionProperty("MSS",property_));

  ContainerProperty *propertyContainer = new ContainerProperty;
  propertyContainer->addProperty(new ChoiceProperty("",property));
  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"isDependent"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input),false)); 
  rotation.setProperty(propertyContainer);

  rotationMapping.setXMLName(MBSIMNS"rotationMapping",false);
  property.clear();
  property.push_back(new TCardanAnglesProperty("V"));
  rotationMapping.setProperty(new ChoiceProperty("",property));

  ombvEditor.setProperty(new OMBVBodySelectionProperty(this));

  weightArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  weightArrow.setXMLName(MBSIMNS"openMBVWeightArrow",false);

  jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  jointForceArrow.setXMLName(MBSIMNS"openMBVJointForceArrow",false);

  jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  jointMomentArrow.setXMLName(MBSIMNS"openMBVJointMomentArrow",false);

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"isFrameOfBodyForRotation"));
  isFrameOfBodyForRotation.setProperty(new ExtPhysicalVarProperty(input)); 
}

int RigidBody::getqRelSize() const {
  int nqT=0, nqR=0;
  if(translation.isActive()) {
    const ChoiceProperty *trans = static_cast<const ChoiceProperty*>(translation.getProperty());
    nqT = static_cast<FunctionProperty*>(trans->getProperty())->getArg1Size();
  }
  if(rotation.isActive()) {
    const ChoiceProperty *rot = static_cast<const ChoiceProperty*>(static_cast<const ContainerProperty*>(rotation.getProperty())->getProperty(0));
    nqR = static_cast<FunctionProperty*>(rot->getProperty())->getArg1Size();
  }
  int nq = nqT + nqR;
  return nq;
}

int RigidBody::getuRelSize() const {
  return getqRelSize();
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
  Frame *f;
  while(e) {
    if(e->ValueStr()==PVNS"embed") {
      TiXmlElement *ee = 0;
      if(e->Attribute("href"))
        f=Frame::readXMLFile(e->Attribute("href"),this);
      else {
        ee = e->FirstChildElement();
        if(ee->ValueStr() == PVNS"localParameter")
          ee = ee->NextSiblingElement();
        f=ObjectFactory::getInstance()->createFrame(ee,this);
      }
      if(f) {
        addFrame(f);
        f->initializeUsingXMLEmbed(e);
        if(ee)
          f->initializeUsingXML(ee);
      }
    }
    else {
      f=ObjectFactory::getInstance()->createFrame(e,this);
      addFrame(f);
      f->initializeUsingXML(e);
    }
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
    FixedRelativeFrame *f=new FixedRelativeFrame("ContourFrame"+toStr(int(contour.size())),this);
    addFrame(f);
    f->initializeUsingXML2(e);
    c->setSavedFrameOfReference(string("../Frame[")+f->getName()+"]");
    e=e->NextSiblingElement();
  }
  while(e) {
    if(e->ValueStr()==PVNS"embed") {
      TiXmlElement *ee = 0;
      if(e->Attribute("href"))
        c=Contour::readXMLFile(e->Attribute("href"),this);
      else {
        ee = e->FirstChildElement();
        if(ee->ValueStr() == PVNS"localParameter")
          ee = ee->NextSiblingElement();
        c=ObjectFactory::getInstance()->createContour(ee,this);
      }
      if(c) {
        addContour(c);
        c->initializeUsingXMLEmbed(e);
        if(ee)
          c->initializeUsingXML(ee);
      }
    }
    else {
      c=ObjectFactory::getInstance()->createContour(e,this);
      if(c) {
        addContour(c);
        c->initializeUsingXML(e);
      }
    }
    e=e->NextSiblingElement();
  }

  K.initializeUsingXML(element);

  mass.initializeUsingXML(element);
  inertia.initializeUsingXML(element);

  translation.initializeUsingXML(element);
  rotation.initializeUsingXML(element);
  rotationMapping.initializeUsingXML(element);

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
  rotationMapping.writeXMLFile(ele0);

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  for(int i=1; i<frame.size(); i++)
    if(frame[i]->isEmbedded())
      frame[i]->writeXMLFileEmbed(ele1);
    else
      frame[i]->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  for(int i=0; i<contour.size(); i++)
    if(contour[i]->isEmbedded())
      contour[i]->writeXMLFileEmbed(ele1);
    else
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
