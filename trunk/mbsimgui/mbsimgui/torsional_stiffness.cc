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
#include "torsional_stiffness.h"
#include "basic_properties.h"
#include "function_properties.h"
#include "kinetics_properties.h"
#include "ombv_properties.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  TorsionalStiffness::TorsionalStiffness(const string &str, Element *parent) : Link(str, parent), coilSpring(0,true), forceArrow(0,false), momentArrow(0,false) {

    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory,MBSIMPOWERTRAIN%"generalizedForceFunction"));

    body1.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIMPOWERTRAIN%"rigidBodyFirstSide"));
    body2.setProperty(new RigidBodyOfReferenceProperty("",this,MBSIMPOWERTRAIN%"rigidBodySecondSide"));

    coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET","",getID()));
    coilSpring.setXMLName(MBSIMPOWERTRAIN%"enableOpenMBVCoilSpring",false);

    forceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    forceArrow.setXMLName(MBSIMPOWERTRAIN%"enableOpenMBVForce",false);

    momentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    momentArrow.setXMLName(MBSIMPOWERTRAIN%"enableOpenMBVMoment",false);
  }

  TorsionalStiffness::~TorsionalStiffness() {
  }

  void TorsionalStiffness::initialize() {
    Link::initialize();
  }

  void TorsionalStiffness::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    Link::initializeUsingXML(element);
    function.initializeUsingXML(element);
    body1.initializeUsingXML(element);
    body2.initializeUsingXML(element);
    coilSpring.initializeUsingXML(element);
    forceArrow.initializeUsingXML(element);
    momentArrow.initializeUsingXML(element);
  }

  DOMElement* TorsionalStiffness::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    function.writeXMLFile(ele0);
    body1.writeXMLFile(ele0);
    body2.writeXMLFile(ele0);
    coilSpring.writeXMLFile(ele0);
    forceArrow.writeXMLFile(ele0);
    momentArrow.writeXMLFile(ele0);
    return ele0;
  }

}
