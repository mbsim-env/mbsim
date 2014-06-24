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
#include "gear.h"
#include "frame.h"
#include "rigidbody.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Gear::Gear(const string &str, Element *parent) : Link(str, parent), function(0,false), gearForceArrow(0,false), gearMomentArrow(0,false) {

    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory,MBSIM%"generalizedForceFunction"));

    dependentBody.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"dependentRigidBody"));

    independentBodies.setProperty(new ListProperty(new GearConstraintPropertyFactory(this),MBSIM%"Transmission"));
    independentBodies.setXMLName(MBSIM%"transmissions");

    gearForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    gearForceArrow.setXMLName(MBSIM%"enableOpenMBVForce",false);

    gearMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    gearMomentArrow.setXMLName(MBSIM%"enableOpenMBVMoment",false);
  }

  void Gear::initialize() {
    Link::initialize();
    dependentBody.initialize();
    independentBodies.initialize();
  }

  void Gear::initializeUsingXML(DOMElement *element) {
    DOMElement *e, *ee;
    Link::initializeUsingXML(element);
    function.initializeUsingXML(element);
    dependentBody.initializeUsingXML(element);
    independentBodies.initializeUsingXML(element);
    gearForceArrow.initializeUsingXML(element);
    gearMomentArrow.initializeUsingXML(element);
  }

  DOMElement* Gear::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    function.writeXMLFile(ele0);
    dependentBody.writeXMLFile(ele0);
    independentBodies.writeXMLFile(ele0);
    gearForceArrow.writeXMLFile(ele0);
    gearMomentArrow.writeXMLFile(ele0);
    return ele0;
  }

}
