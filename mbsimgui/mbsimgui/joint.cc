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
#include "joint.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Joint::Joint(const string &str) : FloatingFrameLink(str), forceDirection(0,false), forceLaw(0,false), momentDirection(0,false), momentLaw(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
    forceDirection.setProperty(new ExtPhysicalVarProperty(input));

    forceLaw.setProperty(new GeneralizedForceLawChoiceProperty(this,MBSIM%"forceLaw"));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
    momentDirection.setProperty(new ExtPhysicalVarProperty(input));

    momentLaw.setProperty(new GeneralizedForceLawChoiceProperty(this,MBSIM%"momentLaw"));
  }

  DOMElement* Joint::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    forceDirection.initializeUsingXML(element);
    forceLaw.initializeUsingXML(element);
    momentDirection.initializeUsingXML(element);
    momentLaw.initializeUsingXML(element);
    return element;
  }

  DOMElement* Joint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FloatingFrameLink::writeXMLFile(parent);
    forceDirection.writeXMLFile(ele0);
    forceLaw.writeXMLFile(ele0);
    momentDirection.writeXMLFile(ele0);
    momentLaw.writeXMLFile(ele0);
    return ele0;
  }

  ElasticJoint::ElasticJoint(const string &str) : FloatingFrameLink(str), forceDirection(0,false), momentDirection(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
    forceDirection.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
    momentDirection.setProperty(new ExtPhysicalVarProperty(input));

    function.setProperty(new ChoiceProperty2(new SpringDamperPropertyFactory(this,"VVV"),MBSIM%"generalizedForceFunction"));
  }

  DOMElement* ElasticJoint::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    forceDirection.initializeUsingXML(element);
    momentDirection.initializeUsingXML(element);
    function.initializeUsingXML(element);
    return element;
  }

  DOMElement* ElasticJoint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FloatingFrameLink::writeXMLFile(parent);
    forceDirection.writeXMLFile(ele0);
    momentDirection.writeXMLFile(ele0);
    function.writeXMLFile(ele0);
    return ele0;
  }

}
