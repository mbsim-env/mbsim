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

OMBVBodyPropertyFactory::OMBVBodyPropertyFactory(const string &ID_) : ID(ID_), count(0) {
  name.push_back(OPENMBVNS"Cube");
  name.push_back(OPENMBVNS"Cuboid");
  name.push_back(OPENMBVNS"Frustum");
  name.push_back(OPENMBVNS"Sphere");
  name.push_back(OPENMBVNS"IvBody");
  name.push_back(OPENMBVNS"CompoundRigidBody");
  name.push_back(OPENMBVNS"InvisibleBody");
}

Property* OMBVBodyPropertyFactory::createProperty(int i) {
  if(i==0)
    return new CubeProperty("Body"+toStr(count++),ID);
  if(i==1)
    return new CuboidProperty("Body"+toStr(count++),ID);
  if(i==2)
    return new FrustumProperty("Body"+toStr(count++),ID);
  if(i==3)
    return new SphereProperty("Body"+toStr(count++),ID);
  if(i==4)
    return new IvBodyProperty("Body"+toStr(count++),ID);
  if(i==5)
    return new CompoundRigidBodyProperty("Body"+toStr(count++),ID);
  if(i==6)
    return new InvisibleBodyProperty("Body"+toStr(count++),ID);
}

void OMBVObjectProperty::writeXMLFileID(TiXmlNode *parent) {
  if(!ID.empty()) {
    TiXmlUnknown *id=new TiXmlUnknown;
    id->SetValue("?OPENMBV_ID "+ID+"?");
    parent->LinkEndChild(id);
  }
}

OMBVFrameProperty::OMBVFrameProperty(const string &name, const string &xmlName_, const std::string &ID) : OMBVObjectProperty(name,ID), xmlName(xmlName_) {

  property.push_back(new ScalarProperty("size","1",LengthUnits()));
  property.push_back(new ScalarProperty("offset","1",NoUnitUnits()));
}

TiXmlElement* OMBVFrameProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement *ee = e->FirstChildElement( MBSIMNS"size" );
    property[0]->initializeUsingXML(ee);
    ee = e->FirstChildElement( MBSIMNS"offset" );
    property[1]->initializeUsingXML(ee);
  //  size.initializeUsingXML(e);
  //  offset.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVFrameProperty::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *e=new TiXmlElement(xmlName);
    writeXMLFileID(e);
    parent->LinkEndChild(e);
    for(int i=0; i<property.size(); i++)
      property[i]->writeXMLFile(e);
  //  size.writeXMLFile(e);
  //  offset.writeXMLFile(e);
    return e;
  }
  else {
    writeXMLFileID(parent);
    TiXmlElement *e=new TiXmlElement(MBSIMNS"size");
    property[0]->writeXMLFile(e);
    parent->LinkEndChild(e);
    e=new TiXmlElement(MBSIMNS"offset");
    property[1]->writeXMLFile(e);
    parent->LinkEndChild(e);
  //  size.writeXMLFile(parent);
  //  offset.writeXMLFile(parent);
    return 0;
  }
}

void OMBVFrameProperty::fromWidget(QWidget *widget) {
 // size.fromWidget(static_cast<OMBVFrameWidget*>(widget)->size);
 // offset.fromWidget(static_cast<OMBVFrameWidget*>(widget)->offset);
}

void OMBVFrameProperty::toWidget(QWidget *widget) {
 // size.toWidget(static_cast<OMBVFrameWidget*>(widget)->size);
 // offset.toWidget(static_cast<OMBVFrameWidget*>(widget)->offset);
}

OMBVDynamicColoredObjectProperty::OMBVDynamicColoredObjectProperty(const string &name, const std::string &ID, bool readXMLType_) : OMBVObjectProperty(name,ID), minimalColorValue(0,false), maximalColorValue(0,false), diffuseColor(0,false), transparency(0,false), readXMLType(readXMLType_) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"), "-", OPENMBVNS"minimalColorValue"));
  minimalColorValue.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"maximalColorValue"));
  maximalColorValue.setProperty(new ExtPhysicalVarProperty(input));

  diffuseColor.setProperty(new ColorProperty(OPENMBVNS"diffuseColor"));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.3"), "-", OPENMBVNS"transparency"));
  transparency.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVDynamicColoredObjectProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=readXMLType?element->FirstChildElement(OPENMBVNS+getType()):element;
  if(e) {
    minimalColorValue.initializeUsingXML(e);
    maximalColorValue.initializeUsingXML(e);
    diffuseColor.initializeUsingXML(e);
    transparency.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVDynamicColoredObjectProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=new TiXmlElement(OPENMBVNS+getType());
  parent->LinkEndChild(e);
  e->SetAttribute("name", name);
  writeXMLFileID(e);
  minimalColorValue.writeXMLFile(e);
  maximalColorValue.writeXMLFile(e);
  diffuseColor.writeXMLFile(e);
  transparency.writeXMLFile(e);
  return e;
}

void OMBVDynamicColoredObjectProperty::fromWidget(QWidget *widget) {
  minimalColorValue.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->minimalColorValue);
  maximalColorValue.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->maximalColorValue);
  diffuseColor.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->diffuseColor);
  transparency.fromWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->transparency);
}

void OMBVDynamicColoredObjectProperty::toWidget(QWidget *widget) {
  minimalColorValue.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->minimalColorValue);
  maximalColorValue.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->maximalColorValue);
  diffuseColor.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->diffuseColor);
  transparency.toWidget(static_cast<OMBVDynamicColoredObjectWidget*>(widget)->transparency);
}

OMBVArrowProperty::OMBVArrowProperty(const string &name, const std::string &ID, bool fromPoint) : OMBVDynamicColoredObjectProperty(name,ID), referencePoint(0,false) {
  readXMLType = true;

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.1"), "m", OPENMBVNS"diameter"));
  diameter.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.2"), "m", OPENMBVNS"headDiameter"));
  headDiameter.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.2"), "m", OPENMBVNS"headLength"));
  headLength.setProperty(new ExtPhysicalVarProperty(input));

  type.setProperty(new TextProperty("","toHead", OPENMBVNS"type", true));

  referencePoint.setProperty(new TextProperty("",fromPoint?"fromPoint":"toPoint", OPENMBVNS"referencePoint", true));
//  if(fromPoint)
//    referencePoint.setActive(true);

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleLength"));
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
  OMBVDynamicColoredObjectProperty::fromWidget(widget);
  diameter.fromWidget(static_cast<OMBVArrowWidget*>(widget)->diameter);
  headDiameter.fromWidget(static_cast<OMBVArrowWidget*>(widget)->headDiameter);
  headLength.fromWidget(static_cast<OMBVArrowWidget*>(widget)->headLength);
  type.fromWidget(static_cast<OMBVArrowWidget*>(widget)->type);
  referencePoint.fromWidget(static_cast<OMBVArrowWidget*>(widget)->referencePoint);
  scaleLength.fromWidget(static_cast<OMBVArrowWidget*>(widget)->scaleLength);
}

void OMBVArrowProperty::toWidget(QWidget *widget) {
  OMBVDynamicColoredObjectProperty::toWidget(widget);
  diameter.toWidget(static_cast<OMBVArrowWidget*>(widget)->diameter);
  headDiameter.toWidget(static_cast<OMBVArrowWidget*>(widget)->headDiameter);
  headLength.toWidget(static_cast<OMBVArrowWidget*>(widget)->headLength);
  type.toWidget(static_cast<OMBVArrowWidget*>(widget)->type);
  referencePoint.toWidget(static_cast<OMBVArrowWidget*>(widget)->referencePoint);
  scaleLength.toWidget(static_cast<OMBVArrowWidget*>(widget)->scaleLength);
}

OMBVCoilSpringProperty::OMBVCoilSpringProperty(const string &name, const std::string &ID) : OMBVObjectProperty(name,ID), crossSectionRadius(0,false), nominalLength(0,false) {

  type.setProperty(new TextProperty("","tube", OPENMBVNS"type", true));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("3"), "-", OPENMBVNS"numberOfCoils"));
  numberOfCoils.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.1"), "m", OPENMBVNS"springRadius"));
  springRadius.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("-1"), "m", OPENMBVNS"crossSectionRadius"));
  crossSectionRadius.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("-1"), "m", OPENMBVNS"nominalLength"));
  nominalLength.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleFactor"));
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

OMBVBodyProperty::OMBVBodyProperty(const string &name, const std::string &ID) : OMBVDynamicColoredObjectProperty(name,ID) {

//  transparency.setActive(true);

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(3), "m", OPENMBVNS"initialTranslation"));
  trans.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(3), "rad", OPENMBVNS"initialRotation"));
  rot.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "-", OPENMBVNS"scaleFactor"));
  scale.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* OMBVBodyProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = OMBVDynamicColoredObjectProperty::initializeUsingXML(element);
  if(e) {
    trans.initializeUsingXML(e);
    rot.initializeUsingXML(e);
    scale.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* OMBVBodyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e=OMBVDynamicColoredObjectProperty::writeXMLFile(parent);
  trans.writeXMLFile(e);
  rot.writeXMLFile(e);
  scale.writeXMLFile(e);
  return e;
}

void OMBVBodyProperty::fromWidget(QWidget *widget) {
  OMBVDynamicColoredObjectProperty::fromWidget(widget);
  trans.fromWidget(static_cast<OMBVBodyWidget*>(widget)->trans);
  rot.fromWidget(static_cast<OMBVBodyWidget*>(widget)->rot);
  scale.fromWidget(static_cast<OMBVBodyWidget*>(widget)->scale);
}

void OMBVBodyProperty::toWidget(QWidget *widget) {
  OMBVDynamicColoredObjectProperty::toWidget(widget);
  trans.toWidget(static_cast<OMBVBodyWidget*>(widget)->trans);
  rot.toWidget(static_cast<OMBVBodyWidget*>(widget)->rot);
  scale.toWidget(static_cast<OMBVBodyWidget*>(widget)->scale);
}

CubeProperty::CubeProperty(const string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"length"));
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

CuboidProperty::CuboidProperty(const string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(getScalars<string>(3,"1")), "m", OPENMBVNS"length"));
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

SphereProperty::SphereProperty(const string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"radius"));
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

FrustumProperty::FrustumProperty(const string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"topRadius"));
  top.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"baseRadius"));
  base.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", OPENMBVNS"height"));
  height.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"), "m", OPENMBVNS"innerTopRadius"));
  innerTop.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"), "m", OPENMBVNS"innerBaseRadius"));
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

IvBodyProperty::IvBodyProperty(const string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {

  ivFileName.setProperty(new FileProperty(OPENMBVNS"ivFileName"));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("-1"), "rad", OPENMBVNS"creaseEdges"));
  creaseEdges.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"), "", OPENMBVNS"boundaryEdges"));
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

CompoundRigidBodyProperty::CompoundRigidBodyProperty(const std::string &name, const std::string &ID) : OMBVBodyProperty(name,ID) {
  bodies.setProperty(new ListProperty(new ChoicePropertyFactory(new OMBVBodyPropertyFactory(ID),"",1),"",0,1));
  //bodies.setXMLName(MBSIMNS"bodies");
}

TiXmlElement* CompoundRigidBodyProperty::initializeUsingXML(TiXmlElement *element) {
  OMBVBodyProperty::initializeUsingXML(element);
  TiXmlElement *e=element->FirstChildElement(OPENMBVNS"scaleFactor");
  TiXmlElement *ee = e->NextSiblingElement();
  bodies.initializeUsingXML(ee);
  return element;
}

TiXmlElement* CompoundRigidBodyProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = OMBVBodyProperty::writeXMLFile(parent);
  bodies.writeXMLFile(ele0);
  return ele0;
}

void CompoundRigidBodyProperty::fromWidget(QWidget *widget) {
  bodies.fromWidget(static_cast<CompoundRigidBodyWidget*>(widget)->bodies);
}

void CompoundRigidBodyProperty::toWidget(QWidget *widget) {
  bodies.toWidget(static_cast<CompoundRigidBodyWidget*>(widget)->bodies);
}

OMBVBodySelectionProperty::OMBVBodySelectionProperty(RigidBody *body) : ombv(0,true), ref(0,false) {
  ombv.setProperty(new ChoiceProperty2(new OMBVBodyPropertyFactory(body->getID()),MBSIMNS"openMBVRigidBody"));
  ref.setProperty(new LocalFrameOfReferenceProperty("Frame[C]",body,MBSIMNS"openMBVFrameOfReference")); 
}

TiXmlElement* OMBVBodySelectionProperty::initializeUsingXML(TiXmlElement *element) {
  ombv.initializeUsingXML(element);
  ref.initializeUsingXML(element);
  return element;
}

TiXmlElement* OMBVBodySelectionProperty::writeXMLFile(TiXmlNode *parent) {
  ombv.writeXMLFile(parent);
  ref.writeXMLFile(parent);
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

OMBVPlaneProperty::OMBVPlaneProperty(const string &xmlName_, const std::string &ID) : OMBVObjectProperty("Plane",ID), xmlName(xmlName_) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIMNS"size"));
  size.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("10"), "", MBSIMNS"numberOfLines"));
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
