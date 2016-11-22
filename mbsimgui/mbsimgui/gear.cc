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
#include "rigid_body.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  GeneralizedGear::GeneralizedGear(const string &str, Element *parent) : RigidBodyLink(str, parent) {

    gearOutput.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIM%"gearOutput"));

    gearInput.setProperty(new ListProperty(new GeneralizedGearConstraintPropertyFactory(this),MBSIM%"gearInput"));

    function.setProperty(new GeneralizedForceLawChoiceProperty(this,MBSIM%"generalizedForceLaw"));
  }

  void GeneralizedGear::initialize() {
    RigidBodyLink::initialize();
    gearOutput.initialize();
    gearInput.initialize();
  }

  DOMElement* GeneralizedGear::initializeUsingXML(DOMElement *element) {
    RigidBodyLink::initializeUsingXML(element);
    gearOutput.initializeUsingXML(element);
    gearInput.initializeUsingXML(element);
    function.initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedGear::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = RigidBodyLink::writeXMLFile(parent);
    gearOutput.writeXMLFile(ele0);
    gearInput.writeXMLFile(ele0);
    function.writeXMLFile(ele0);
    return ele0;
  }

}
