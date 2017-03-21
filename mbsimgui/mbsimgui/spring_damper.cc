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

  SpringDamper::SpringDamper(const string &str) : FrameLink(str), coilSpring(0,true) {

    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));

    coilSpring.setProperty(new CoilSpringMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVCoilSpring",getID()));
  }

  DOMElement* SpringDamper::initializeUsingXML(DOMElement *element) {
    FrameLink::initializeUsingXML(element);
    forceFunction.initializeUsingXML(element);
    unloadedLength.initializeUsingXML(element);
    coilSpring.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpringDamper::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FrameLink::writeXMLFile(parent);
    forceFunction.writeXMLFile(ele0);
    unloadedLength.writeXMLFile(ele0);
    coilSpring.writeXMLFile(ele0);
    return ele0;
  }

  DirectionalSpringDamper::DirectionalSpringDamper(const string &str) : FloatingFrameLink(str), coilSpring(0,true) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(3),"-",MBSIM%"forceDirection"));
    forceDirection.setProperty(new ExtPhysicalVarProperty(input));

    forceFunction.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"forceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"unloadedLength",vector<string>(2,"m")),"",4));

    coilSpring.setProperty(new CoilSpringMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVCoilSpring",getID()));
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

  GeneralizedSpringDamper::GeneralizedSpringDamper(const string &str) : DualRigidBodyLink(str) {

    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this),MBSIM%"generalizedForceFunction"));

    unloadedLength.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("0",MBSIM%"generalizedUnloadedLength",vector<string>(2)),"",4));
  }

  DOMElement* GeneralizedSpringDamper::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    function.initializeUsingXML(element);
    unloadedLength.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedSpringDamper::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = DualRigidBodyLink::writeXMLFile(parent);
    function.writeXMLFile(ele0);
    unloadedLength.writeXMLFile(ele0);
    return ele0;
  }

}
