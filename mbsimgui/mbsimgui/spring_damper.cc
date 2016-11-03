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

namespace MBSimGUI {

  SpringDamper::SpringDamper(const string &str, Element *parent) : Link(str, parent), coilSpring(0,true), forceArrow(0,false) {

    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));

    connections.setProperty(new ConnectFramesProperty(2,this));

    coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
    coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);

    forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);
  }

  void SpringDamper::initialize() {
    Link::initialize();
    connections.initialize();
  }

  DOMElement* SpringDamper::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    forceFunction.initializeUsingXML(element);
    unloadedLength.initializeUsingXML(element);
    connections.initializeUsingXML(element);
    coilSpring.initializeUsingXML(element);
    forceArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpringDamper::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    forceFunction.writeXMLFile(ele0);
    unloadedLength.writeXMLFile(ele0);
    connections.writeXMLFile(ele0);
    coilSpring.writeXMLFile(ele0);
    forceArrow.writeXMLFile(ele0);
    return ele0;
  }

  DirectionalSpringDamper::DirectionalSpringDamper(const string &str, Element *parent) : FloatingFrameLink(str, parent), coilSpring(0,true) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(3),"-",MBSIM%"forceDirection"));
    forceDirection.setProperty(new ExtPhysicalVarProperty(input));

    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));

    coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
    coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);
  }

  DOMElement* DirectionalSpringDamper::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    forceDirection.initializeUsingXML(element);
    forceFunction.initializeUsingXML(element);
    unloadedLength.initializeUsingXML(element);
    coilSpring.initializeUsingXML(element);
    return element;
  }

  DOMElement* DirectionalSpringDamper::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FloatingFrameLink::writeXMLFile(parent);
    forceDirection.writeXMLFile(ele0);
    forceFunction.writeXMLFile(ele0);
    unloadedLength.writeXMLFile(ele0);
    coilSpring.writeXMLFile(ele0);
    return ele0;
  }

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &str, Element *parent) : RigidBodyLink(str, parent), body1(0,false), coilSpring(0,true), forceArrow(0,false), momentArrow(0,false) {

    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"generalizedForceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIM%"unloadedGeneralizedLength",vector<string>(2)),"",4));

    body1.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBodyFirstSide"));
    body2.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"rigidBodySecondSide"));

    coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
    coilSpring.setXMLName(MBSIM%"enableOpenMBVCoilSpring",false);

    forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    forceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    momentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    momentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  DOMElement* GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    function.initializeUsingXML(element);
    unloadedLength.initializeUsingXML(element);
    body1.initializeUsingXML(element);
    body2.initializeUsingXML(element);
    coilSpring.initializeUsingXML(element);
    forceArrow.initializeUsingXML(element);
    momentArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedSpringDamper::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = RigidBodyLink::writeXMLFile(parent);
    function.writeXMLFile(ele0);
    unloadedLength.writeXMLFile(ele0);
    body1.writeXMLFile(ele0);
    body2.writeXMLFile(ele0);
    coilSpring.writeXMLFile(ele0);
    forceArrow.writeXMLFile(ele0);
    momentArrow.writeXMLFile(ele0);
    return ele0;
  }

}
