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
#include "kinematic_functions_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "kinematics_properties.h"

#define iK 1
#define im 2
#define ii 3
#define ifi 4
#define it 5
#define io 6
#define ifo 7

using namespace std;
using namespace MBXMLUtils;

RigidBody::RigidBody(const string &str, Element *parent) : Body(str,parent), constrained(false), K(0,false), frameForInertiaTensor(0,false), translation(0,false), rotation(0,false), translationDependentRotation(0,false), coordinateTransformationForRotation(0,false), ombvEditor(0,true), weightArrow(0,false), jointForceArrow(0,false), jointMomentArrow(0,false) {
  Frame *C = new Frame("C",this);
  type = "RigidBody";
  addFrame(C);

  property.push_back(new LocalFrameOfReferenceProperty("frameForKinematics","Frame[C]",this)); 
  property[iK]->setDisabling(true);
  property[iK]->setDisabled(true);

  property.push_back(new Scalar_Property("mass",MassUnits()));

  property.push_back(new SymMat_Property("inertiaTensor",InertiaUnits()));

  property.push_back(new LocalFrameOfReferenceProperty("frameForInertiaTensor","Frame[C]",this)); 
  property[ifi]->setDisabling(true);
  property[ifi]->setDisabled(true);

//  property.push_back(new StateDependentTranslation("stateDependentTranslation"));
  property.push_back(new Translation("stateDependentTranslation"));
  property[it]->setDisabling(true);
  property[it]->setDisabled(true);

  rotation.setProperty(new ChoiceProperty2(new RotationPropertyFactory4,"",3)); 

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"translationDependentRotation"));
  translationDependentRotation.setProperty(new ExtPhysicalVarProperty(input)); 
  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"coordinateTransformationForRotation"));
  coordinateTransformationForRotation.setProperty(new ExtPhysicalVarProperty(input)); 

  property.push_back(new OpenMBVRigidBodyChoiceProperty("openMBVRigidBody"));
  property[io]->setDisabling(true);
  OMBVBodyFactory factory;
  property[io]->addProperty(factory.createBody(0,ID));

  property.push_back(new LocalFrameOfReferenceProperty("openMBVFrameOfReference","Frame[C]",this)); 
  property[ifo]->setDisabling(true);
  property[ifo]->setDisabled(true);

  weightArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  weightArrow.setXMLName(MBSIMNS"openMBVWeightArrow",false);

  jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  jointForceArrow.setXMLName(MBSIMNS"openMBVJointForceArrow",false);

  jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  jointMomentArrow.setXMLName(MBSIMNS"openMBVJointMomentArrow",false);

}

int RigidBody::getqRelSize() const {
  int nqT=0, nqR=0;
  //if(translation.isActive()) {
  //  const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(translation.getProperty())->getProperty());
  //  const ChoiceProperty2 *trans = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
  //  nqT = static_cast<FunctionProperty*>(trans->getProperty())->getArg1Size();
  //}
  //if(rotation.isActive()) {
  //  const ExtProperty *extProperty = static_cast<const ExtProperty*>(static_cast<const ChoiceProperty2*>(rotation.getProperty())->getProperty());
  //  const ChoiceProperty2 *rot = static_cast<const ChoiceProperty2*>(extProperty->getProperty());
  //  nqR = static_cast<FunctionProperty*>(rot->getProperty())->getArg1Size();
  //}
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

  TiXmlElement *ele1 = element->FirstChildElement( MBSIMNS"frameForKinematics" );
  if(ele1) {
    property[iK]->initializeUsingXML(ele1);
    property[iK]->setDisabled(false);
  }

  ele1 = element->FirstChildElement( MBSIMNS"mass" );
  property[im]->initializeUsingXML(ele1);
  ele1 = element->FirstChildElement( MBSIMNS"inertiaTensor" );
  property[ii]->initializeUsingXML(ele1);
  ele1 = element->FirstChildElement( MBSIMNS"frameForInertiaTensor" );
  if(ele1) {
    property[ifi]->initializeUsingXML(ele1);
    property[ifi]->setDisabled(false);
  }

  ele1 = element->FirstChildElement( MBSIMNS"stateDependentTranslation" );
  if(ele1) {
    property[it]->initializeUsingXML(ele1);
    property[it]->setDisabled(false);
  }
  ele1 = element->FirstChildElement( MBSIMNS"timeDependentTranslation" );
  if(ele1) {
    property[it]->setName("timeDependentTranslation");
    FunctionFactory *factory = new FunctionFactory2;
    FunctionProperty* function = factory->createFunction(0);
    function->setFactory(factory);
    property[it]->setProperty(function);
    property[it]->initializeUsingXML(ele1);
    property[it]->setDisabled(false);
  }

//  translation.initializeUsingXML(element);
//  rotation.initializeUsingXML(element);
//  translationDependentRotation.initializeUsingXML(element);
//  coordinateTransformationForRotation.initializeUsingXML(element);
//
  ele1 = element->FirstChildElement( MBSIMNS"openMBVRigidBody" );
  if(ele1) {
    OMBVBodyFactory factory;
    delete property[io]->getProperty();
    property[io]->setProperty(factory.createBody(ele1->FirstChildElement(),ID));
    property[io]->initializeUsingXML(ele1->FirstChildElement());
    property[io]->setDisabled(false);
  }
//
////  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameC");
////  if(e)
////    getFrame(0)->initializeUsingXML2(e);
////  else
////    getFrame(0)->setOpenMBVFrame(false);
//
//  weightArrow.initializeUsingXML(element);
//
//  jointForceArrow.initializeUsingXML(element);
//  jointMomentArrow.initializeUsingXML(element);
//
//  Body::initializeUsingXML(element);
}

TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Body::writeXMLFile(parent);
  TiXmlElement *ele1;

  if(not(property[iK]->isDisabled())) {
    ele1 = new TiXmlElement( MBSIMNS"frameForKinematics" );
    property[iK]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }

  ele1 = new TiXmlElement( MBSIMNS"mass" );
  property[im]->writeXMLFile(ele1);
  ele0->LinkEndChild(ele1);
  ele1 = new TiXmlElement( MBSIMNS"inertiaTensor" );
  property[ii]->writeXMLFile(ele1);
  ele0->LinkEndChild(ele1);
  if(not(property[ifi]->isDisabled())) {
    ele1 = new TiXmlElement( MBSIMNS"frameForInertiaTensor" );
    property[ifi]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }

  if(not(property[it]->isDisabled())) {
    ele1 = new TiXmlElement( MBSIMNS+property[it]->getName() );
    property[it]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }

  rotation.writeXMLFile(ele0);
  translationDependentRotation.writeXMLFile(ele0);
  coordinateTransformationForRotation.writeXMLFile(ele0);

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

  if(not(property[io]->isDisabled())) {
    ele1 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
    property[io]->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
//  if(not(property[6]->isDisabled())) {
//    ele1 = new TiXmlElement( MBSIMNS"openMBVFrameOfReference" );
//    property[6]->writeXMLFile(ele1);
//    ele0->LinkEndChild(ele1);
//  }

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
