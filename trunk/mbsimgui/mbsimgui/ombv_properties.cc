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
#include "ombv_properties.h"
#include "variable_properties.h"
#include "extended_properties.h"
#include "ombv_widgets.h"
#include "rigidbody.h"
#include "frame.h"

using namespace std;
using namespace MBXMLUtils;

void OMBVObjectProperty::writeXMLFileID(TiXmlNode *parent) {
  if(!ID.empty()) {
    TiXmlUnknown *id=new TiXmlUnknown;
    id->SetValue("?OPENMBV_ID "+ID+"?");
    parent->LinkEndChild(id);
  }
}

OMBVFrameProperty::OMBVFrameProperty(const string &name, const string &xmlName_) : OMBVObjectProperty(name), xmlName(xmlName_) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIMNS"size"));
  size.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "-", MBSIMNS"offset"));
  offset.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVFrameProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    size.initializeUsingXML(e);
    offset.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVFrameProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *e=new TiXmlElement(xmlName);
    writeXMLFileID(e);
    parent->LinkEndChild(e);
    size.writeXMLFile(e);
    offset.writeXMLFile(e);
    return e;
  }
  else {
    writeXMLFileID(parent);
    size.writeXMLFile(parent);
    offset.writeXMLFile(parent);
    return 0;
  }
}

void OMBVFrameProperty::fromWidget(QWidget *widget) {
  size.fromWidget(static_cast<OMBVFrameWidget*>(widget)->size);
  offset.fromWidget(static_cast<OMBVFrameWidget*>(widget)->offset);
}

void OMBVFrameProperty::toWidget(QWidget *widget) {
  size.toWidget(static_cast<OMBVFrameWidget*>(widget)->size);
  offset.toWidget(static_cast<OMBVFrameWidget*>(widget)->offset);
}

OMBVDynamicColoredObjectProperty::OMBVDynamicColoredObjectProperty(const string &name) : OMBVObjectProperty(name), minimalColorValue(0,false), maximalColorValue(0,false), staticColor(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "-", OPENMBVNS"minimalColorValue"));
  minimalColorValue.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"maximalColorValue"));
  maximalColorValue.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "-", OPENMBVNS"staticColor"));
  staticColor.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVDynamicColoredObjectProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType());
  if(e) {
    minimalColorValue.initializeUsingXML(e);
    maximalColorValue.initializeUsingXML(e);
    staticColor.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVDynamicColoredObjectProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  writeXMLFileID(e);
  minimalColorValue.writeXMLFile(e);
  maximalColorValue.writeXMLFile(e);
  staticColor.writeXMLFile(e);
  return e;
}

void OMBVDynamicColoredObjectProperty::fromWidget(QWidget *widget) {
  minimalColorValue.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->minimalColorValue);
  maximalColorValue.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->maximalColorValue);
  staticColor.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->staticColor);
}

void OMBVDynamicColoredObjectProperty::toWidget(QWidget *widget) {
  minimalColorValue.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->minimalColorValue);
  maximalColorValue.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->maximalColorValue);
  staticColor.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->staticColor);
}

OMBVArrowProperty::OMBVArrowProperty(const string &name, bool fromPoint) : OMBVDynamicColoredObjectProperty(name), referencePoint(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.1"), "m", OPENMBVNS"diameter"));
  diameter.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.2"), "m", OPENMBVNS"headDiameter"));
  headDiameter.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.2"), "m", OPENMBVNS"headLength"));
  headLength.setProperty(new ExtPhysicalVarProperty(input));

  type.setProperty(new TextProperty("toHead", OPENMBVNS"type", true));

  referencePoint.setProperty(new TextProperty(fromPoint?"fromPoint":"toPoint", OPENMBVNS"referencePoint", true));
  if(fromPoint)
    referencePoint.setActive(true);

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleLength"));
  scaleLength.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVArrowProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = OMBVDynamicColoredObjectProperty::initializeUsingXML(element);
  if(e) {
    diameter.initializeUsingXML(e);
    headDiameter.initializeUsingXML(e);
    headLength.initializeUsingXML(e);
    type.initializeUsingXML(e);
    referencePoint.initializeUsingXML(e);
    scaleLength.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVArrowProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVDynamicColoredObjectProperty::writeXMLFile(parent);
  diameter.writeXMLFile(e);
  headDiameter.writeXMLFile(e);
  headLength.writeXMLFile(e);
  type.writeXMLFile(e);
  referencePoint.writeXMLFile(e);
  scaleLength.writeXMLFile(e);
  return e;
}

void OMBVArrowProperty::fromWidget(QWidget *widget) {
  diameter.fromWidget(static_cast<OMBVArrowWidget*>(widget)->diameter);
  headDiameter.fromWidget(static_cast<OMBVArrowWidget*>(widget)->headDiameter);
  headLength.fromWidget(static_cast<OMBVArrowWidget*>(widget)->headLength);
  type.fromWidget(static_cast<OMBVArrowWidget*>(widget)->type);
  referencePoint.fromWidget(static_cast<OMBVArrowWidget*>(widget)->referencePoint);
  scaleLength.fromWidget(static_cast<OMBVArrowWidget*>(widget)->scaleLength);
}

void OMBVArrowProperty::toWidget(QWidget *widget) {
  diameter.toWidget(static_cast<OMBVArrowWidget*>(widget)->diameter);
  headDiameter.toWidget(static_cast<OMBVArrowWidget*>(widget)->headDiameter);
  headLength.toWidget(static_cast<OMBVArrowWidget*>(widget)->headLength);
  type.toWidget(static_cast<OMBVArrowWidget*>(widget)->type);
  referencePoint.toWidget(static_cast<OMBVArrowWidget*>(widget)->referencePoint);
  scaleLength.toWidget(static_cast<OMBVArrowWidget*>(widget)->scaleLength);
}

OMBVCoilSpringProperty::OMBVCoilSpringProperty(const string &name) : OMBVObjectProperty(name), crossSectionRadius(0,false), nominalLength(0,false) {

  type.setProperty(new TextProperty("tube", OPENMBVNS"type", true));

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("3"), "-", OPENMBVNS"numberOfCoils"));
  numberOfCoils.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.1"), "m", OPENMBVNS"springRadius"));
  springRadius.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("-1"), "m", OPENMBVNS"crossSectionRadius"));
  crossSectionRadius.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("-1"), "m", OPENMBVNS"nominalLength"));
  nominalLength.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleFactor"));
  scaleFactor.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVCoilSpringProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS+getType());
  if(e) {
    type.initializeUsingXML(e);
    numberOfCoils.initializeUsingXML(e);
    springRadius.initializeUsingXML(e);
    crossSectionRadius.initializeUsingXML(e);
    nominalLength.initializeUsingXML(e);
    scaleFactor.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVCoilSpringProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType());
  parent->LinkEndChild(e);
  e->SetAttribute("name", "dummy");
  writeXMLFileID(e);
  type.writeXMLFile(e);
  numberOfCoils.writeXMLFile(e);
  springRadius.writeXMLFile(e);
  crossSectionRadius.writeXMLFile(e);
  nominalLength.writeXMLFile(e);
  scaleFactor.writeXMLFile(e);
  return e;
}

void OMBVCoilSpringProperty::fromWidget(QWidget *widget) {
  type.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->type);
  numberOfCoils.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->numberOfCoils);
  springRadius.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->springRadius);
  crossSectionRadius.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->crossSectionRadius);
  nominalLength.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->nominalLength);
  scaleFactor.fromWidget(static_cast<OMBVCoilSpringWidget*>(widget)->scaleFactor);
}

void OMBVCoilSpringProperty::toWidget(QWidget *widget) {
  type.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->type);
  numberOfCoils.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->numberOfCoils);
  springRadius.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->springRadius);
  crossSectionRadius.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->crossSectionRadius);
  nominalLength.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->nominalLength);
  scaleFactor.toWidget(static_cast<OMBVCoilSpringWidget*>(widget)->scaleFactor);
}

OMBVBodyProperty::OMBVBodyProperty(const string &name) : OMBVObjectProperty(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "-", OPENMBVNS"staticColor"));
  color.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(3), "m", OPENMBVNS"initialTranslation"));
  trans.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(3), "rad", OPENMBVNS"initialRotation"));
  rot.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleFactor"));
  scale.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVBodyProperty::initializeUsingXML(TiXmlElement *element) {
  color.initializeUsingXML(element);
  trans.initializeUsingXML(element);
  rot.initializeUsingXML(element);
  scale.initializeUsingXML(element);
  return element;
}

TiXmlElement* OMBVBodyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType());
  parent->LinkEndChild(e);
  e->SetAttribute("name", name==""?"NOTSET":name);
  writeXMLFileID(e);
  color.writeXMLFile(e);
  trans.writeXMLFile(e);
  rot.writeXMLFile(e);
  scale.writeXMLFile(e);
  return e;
}

void OMBVBodyProperty::fromWidget(QWidget *widget) {
  color.fromWidget(static_cast<OMBVBodyWidget*>(widget)->color);
  trans.fromWidget(static_cast<OMBVBodyWidget*>(widget)->trans);
  rot.fromWidget(static_cast<OMBVBodyWidget*>(widget)->rot);
  scale.fromWidget(static_cast<OMBVBodyWidget*>(widget)->scale);
}

void OMBVBodyProperty::toWidget(QWidget *widget) {
  color.toWidget(static_cast<OMBVBodyWidget*>(widget)->color);
  trans.toWidget(static_cast<OMBVBodyWidget*>(widget)->trans);
  rot.toWidget(static_cast<OMBVBodyWidget*>(widget)->rot);
  scale.toWidget(static_cast<OMBVBodyWidget*>(widget)->scale);
}

CubeProperty::CubeProperty(const string &name) : OMBVBodyProperty(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"length"));
  length.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* CubeProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  length.initializeUsingXML(element);
  return element;
}

TiXmlElement* CubeProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  length.writeXMLFile(e);
  return e;
}

void CubeProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  length.fromWidget(static_cast<CubeWidget*>(widget)->length);
}

void CubeProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  length.toWidget(static_cast<CubeWidget*>(widget)->length);
}

CuboidProperty::CuboidProperty(const string &name) : OMBVBodyProperty(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(getScalars<string>(3,"1")), "m", OPENMBVNS"length"));
  length.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* CuboidProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  length.initializeUsingXML(element);
  return element;
}

TiXmlElement* CuboidProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  length.writeXMLFile(e);
  return e;
}

void CuboidProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  length.fromWidget(static_cast<CuboidWidget*>(widget)->length);
}

void CuboidProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  length.toWidget(static_cast<CuboidWidget*>(widget)->length);
}

SphereProperty::SphereProperty(const string &name) : OMBVBodyProperty(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"radius"));
  radius.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SphereProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  radius.initializeUsingXML(element);
  return element;
}

TiXmlElement* SphereProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  radius.writeXMLFile(e);
  return e;
}

void SphereProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  radius.fromWidget(static_cast<SphereWidget*>(widget)->radius);
}

void SphereProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  radius.toWidget(static_cast<SphereWidget*>(widget)->radius);
}

FrustumProperty::FrustumProperty(const string &name) : OMBVBodyProperty(name) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"topRadius"));
  top.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"baseRadius"));
  base.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"height"));
  height.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "m", OPENMBVNS"innerTopRadius"));
  innerTop.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "m", OPENMBVNS"innerBaseRadius"));
  innerBase.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* FrustumProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  TiXmlElement *e;
  base.initializeUsingXML(element);
  top.initializeUsingXML(element);
  height.initializeUsingXML(element);
  innerBase.initializeUsingXML(element);
  innerTop.initializeUsingXML(element);
  return element;
}

TiXmlElement* FrustumProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  base.writeXMLFile(e);
  top.writeXMLFile(e);
  height.writeXMLFile(e);
  innerBase.writeXMLFile(e);
  innerTop.writeXMLFile(e);
  return e;
}

void FrustumProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  base.fromWidget(static_cast<FrustumWidget*>(widget)->base);
  top.fromWidget(static_cast<FrustumWidget*>(widget)->top);
  height.fromWidget(static_cast<FrustumWidget*>(widget)->height);
  innerBase.fromWidget(static_cast<FrustumWidget*>(widget)->innerBase);
  innerTop.fromWidget(static_cast<FrustumWidget*>(widget)->innerTop);
}

void FrustumProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  base.toWidget(static_cast<FrustumWidget*>(widget)->base);
  top.toWidget(static_cast<FrustumWidget*>(widget)->top);
  height.toWidget(static_cast<FrustumWidget*>(widget)->height);
  innerBase.toWidget(static_cast<FrustumWidget*>(widget)->innerBase);
  innerTop.toWidget(static_cast<FrustumWidget*>(widget)->innerTop);
}

IvBodyProperty::IvBodyProperty(const string &name) : OMBVBodyProperty(name) {

  ivFileName.setProperty(new FileProperty(OPENMBVNS"ivFileName"));

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("-1"), "rad", OPENMBVNS"creaseEdges"));
  creaseEdges.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"), "", OPENMBVNS"boundaryEdges"));
  boundaryEdges.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* IvBodyProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  ivFileName.initializeUsingXML(element);
  creaseEdges.initializeUsingXML(element);
  boundaryEdges.initializeUsingXML(element);
  return element;
}

TiXmlElement* IvBodyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  ivFileName.writeXMLFile(e);
  creaseEdges.writeXMLFile(e);
  boundaryEdges.writeXMLFile(e);
  return e;
}

void IvBodyProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  ivFileName.fromWidget(static_cast<IvBodyWidget*>(widget)->ivFileName);
  creaseEdges.fromWidget(static_cast<IvBodyWidget*>(widget)->creaseEdges);
  boundaryEdges.fromWidget(static_cast<IvBodyWidget*>(widget)->boundaryEdges);
}

void IvBodyProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  ivFileName.toWidget(static_cast<IvBodyWidget*>(widget)->ivFileName);
  creaseEdges.toWidget(static_cast<IvBodyWidget*>(widget)->creaseEdges);
  boundaryEdges.toWidget(static_cast<IvBodyWidget*>(widget)->boundaryEdges);
}

TiXmlElement* CompoundRigidBodyProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS"scaleFactor");
  e=e->NextSiblingElement();
  while(e) {
    body.push_back(new OMBVBodyChoiceProperty("Body"+toStr(int(body.size()+1)),false));
    body[body.size()-1]->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
  return e;
}

TiXmlElement* CompoundRigidBodyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVBodyProperty::writeXMLFile(parent);
  for(unsigned int i=0; i<body.size(); i++)
    body[i]->writeXMLFile(e);
  return e;
}

void CompoundRigidBodyProperty::fromWidget(QWidget *widget) {
  OMBVBodyProperty::fromWidget(widget);
  for(unsigned int i=0; i<static_cast<CompoundRigidBodyWidget*>(widget)->body.size(); i++) {
    body.push_back(new OMBVBodyChoiceProperty("Body"+toStr(int(body.size()+1)),false));
    body[i]->fromWidget(static_cast<CompoundRigidBodyWidget*>(widget)->body[i]);
  }
}

void CompoundRigidBodyProperty::toWidget(QWidget *widget) {
  OMBVBodyProperty::toWidget(widget);
  for(unsigned int i=0; i<body.size(); i++) {
    static_cast<CompoundRigidBodyWidget*>(widget)->addBody();
    body[i]->toWidget(static_cast<CompoundRigidBodyWidget*>(widget)->body[i]);
  }
}

OMBVBodyChoiceProperty::OMBVBodyChoiceProperty(const string &name_, bool flag, const string &ID_) : ombv(0), index(0), name(name_), ID(ID_) {
  ombvSelection(0);
}

void OMBVBodyChoiceProperty::ombvSelection(int index_) {
  index = index_;
  delete ombv;
  if(index==0)
    ombv = new CubeProperty(name);  
  if(index==1)
    ombv = new CuboidProperty(name);  
  else if(index==2)
    ombv = new FrustumProperty(name);  
  else if(index==3)
    ombv = new SphereProperty(name);  
  else if(index==4)
    ombv = new IvBodyProperty(name);  
  else if(index==5)
    ombv = new CompoundRigidBodyProperty(name);  
  else if(index==6)
    ombv = new InvisibleBodyProperty(name);  
  ombv->setID(ID);
}

TiXmlElement* OMBVBodyChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  //TiXmlElement *e1 = element->FirstChildElement();
  TiXmlElement *e1 = element;
  if(e1) {
    index = 0;
    if(e1->ValueStr() == OPENMBVNS"Cube")
      index = 0;
    if(e1->ValueStr() == OPENMBVNS"Cuboid")
      index = 1;
    else if(e1->ValueStr() == OPENMBVNS"Frustum")
      index = 2;
    else if(e1->ValueStr() == OPENMBVNS"Sphere")
      index = 3;
    else if(e1->ValueStr() == OPENMBVNS"IvBody")
      index = 4;
    else if(e1->ValueStr() == OPENMBVNS"CompoundRigidBody")
      index = 5;
    else if(e1->ValueStr() == OPENMBVNS"InvisibleBody")
      index = 6;
    ombvSelection(index);
    ombv->initializeUsingXML(e1);
  }
  return e1;
}

TiXmlElement* OMBVBodyChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  ombv->writeXMLFile(parent);
  return 0;
}

void OMBVBodyChoiceProperty::fromWidget(QWidget *widget) {
  ombvSelection(static_cast<OMBVBodyChoiceWidget*>(widget)->comboBox->currentIndex());
  ombv->fromWidget(static_cast<OMBVBodyChoiceWidget*>(widget)->ombv);
}

void OMBVBodyChoiceProperty::toWidget(QWidget *widget) {
  static_cast<OMBVBodyChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<OMBVBodyChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<OMBVBodyChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<OMBVBodyChoiceWidget*>(widget)->ombvSelection(index);
  ombv->toWidget(static_cast<OMBVBodyChoiceWidget*>(widget)->ombv);
}

OMBVBodySelectionProperty::OMBVBodySelectionProperty(RigidBody *body) : ombv("NOTSET", true, body->getID()), ref("Frame[C]",body,MBSIMNS"frameOfReference") {

}

TiXmlElement* OMBVBodySelectionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(MBSIMNS"openMBVRigidBody");
  if(e) {
    ombv.initializeUsingXML(e->FirstChildElement());
    ref.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVBodySelectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement( MBSIMNS"openMBVRigidBody" );
  ombv.writeXMLFile(ele0);
  if(ref.getFrame()!="Frame[C]")
    ref.writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  return 0;
}

void OMBVBodySelectionProperty::fromWidget(QWidget *widget) {
  ref.fromWidget(static_cast<OMBVBodySelectionWidget*>(widget)->ref);
  ombv.fromWidget(static_cast<OMBVBodySelectionWidget*>(widget)->ombv);
}

void OMBVBodySelectionProperty::toWidget(QWidget *widget) {
  ref.toWidget(static_cast<OMBVBodySelectionWidget*>(widget)->ref);
  ombv.toWidget(static_cast<OMBVBodySelectionWidget*>(widget)->ombv);
}

TiXmlElement* OMBVEmptyProperty::initializeUsingXML(TiXmlElement *parent) {
  return parent->FirstChildElement(xmlName);
}

TiXmlElement* OMBVEmptyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = new TiXmlElement(xmlName);
  writeXMLFileID(ele);
  parent->LinkEndChild(ele);
  return 0;
}

OMBVPlaneProperty::OMBVPlaneProperty(const string &xmlName_) : OMBVObjectProperty("Plane"), xmlName(xmlName_) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIMNS"size"));
  size.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("10"), "", MBSIMNS"numberOfLines"));
  numberOfLines.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVPlaneProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    size.initializeUsingXML(e);
    numberOfLines.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVPlaneProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(xmlName);
  writeXMLFileID(e);
  parent->LinkEndChild(e);
  size.writeXMLFile(e);
  numberOfLines.writeXMLFile(e);
  return e;
}

void OMBVPlaneProperty::fromWidget(QWidget *widget) {
  size.fromWidget(static_cast<OMBVPlaneWidget*>(widget)->size);
  numberOfLines.fromWidget(static_cast<OMBVPlaneWidget*>(widget)->numberOfLines);
}

void OMBVPlaneProperty::toWidget(QWidget *widget) {
  size.toWidget(static_cast<OMBVPlaneWidget*>(widget)->size);
  numberOfLines.toWidget(static_cast<OMBVPlaneWidget*>(widget)->numberOfLines);
}
