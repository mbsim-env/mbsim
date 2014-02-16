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
#include "spring_damper.h"
#include "basic_properties.h"
#include "function_properties.h"
#include "kinetics_properties.h"
#include "ombv_properties.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

SpringDamper::SpringDamper(const string &str, Element *parent) : Link(str, parent), coilSpring(0,true), forceArrow(0,false) {

  forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory,MBSIM%"forceFunction"));

  connections.setProperty(new ConnectFramesProperty(2,this));

  coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
  coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);
}

SpringDamper::~SpringDamper() {
}

void SpringDamper::initialize() {
  Link::initialize();
  connections.initialize();
}

void SpringDamper::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  Link::initializeUsingXML(element);
  forceFunction.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  coilSpring.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
}

DOMElement* SpringDamper::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Link::writeXMLFile(parent);
  forceFunction.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  coilSpring.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  return ele0;
}

DirectionalSpringDamper::DirectionalSpringDamper(const string &str, Element *parent) : Link(str, parent), coilSpring(0,true), forceArrow(0,false) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(3),"-",MBSIM%"forceDirection"));
  forceDirection.setProperty(new ExtPhysicalVarProperty(input));

  forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory,MBSIM%"forceFunction"));

  connections.setProperty(new ConnectFramesProperty(2,this));

  coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
  coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);
}

DirectionalSpringDamper::~DirectionalSpringDamper() {
}

void DirectionalSpringDamper::initialize() {
  Link::initialize();
  connections.initialize();
}

void DirectionalSpringDamper::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  Link::initializeUsingXML(element);
  forceDirection.initializeUsingXML(element);
  forceFunction.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  coilSpring.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
}

DOMElement* DirectionalSpringDamper::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Link::writeXMLFile(parent);
  forceDirection.writeXMLFile(ele0);
  forceFunction.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  coilSpring.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  return ele0;
}

GeneralizedSpringDamper::GeneralizedSpringDamper(const string &str, Element *parent) : Link(str, parent), coilSpring(0,true), forceArrow(0,false), momentArrow(0,false) {

  function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory,MBSIM%"generalizedForceFunction"));

  body.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBody"));

  coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
  coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  momentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
}

GeneralizedSpringDamper::~GeneralizedSpringDamper() {
}

void GeneralizedSpringDamper::initialize() {
  Link::initialize();
}

void GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  Link::initializeUsingXML(element);
  function.initializeUsingXML(element);
  body.initializeUsingXML(element);
  coilSpring.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
  momentArrow.initializeUsingXML(element);
}

DOMElement* GeneralizedSpringDamper::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Link::writeXMLFile(parent);
  function.writeXMLFile(ele0);
  body.writeXMLFile(ele0);
  coilSpring.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  momentArrow.writeXMLFile(ele0);
  return ele0;
}
