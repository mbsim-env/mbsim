/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2013 Martin Förg

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
#include "actuator.h"
#include "ombv_properties.h"
#include "kinetics_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Actuator::Actuator(const string &str, Element *parent) : Link(str, parent), forceDir(0,false), momentDir(0,false), frameOfReference(0,false), actuatorForceArrow(0,false), actuatorMomentArrow(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMCONTROL%"forceDirection"));
    forceDir.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMCONTROL%"momentDirection"));
    momentDir.setProperty(new ExtPhysicalVarProperty(input));

    frameOfReference.setProperty(new IntegerProperty(1,MBSIMCONTROL%"referenceFrame")); 

    inputSignal.setProperty(new SignalOfReferenceProperty("",this, MBSIMCONTROL%"inputSignal"));

    connections.setProperty(new ConnectFramesProperty(2,this,MBSIMCONTROL%"connect"));

    actuatorForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    actuatorForceArrow.setXMLName(MBSIMCONTROL%"enableOpenMBVForce",false);

    actuatorMomentArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
    actuatorMomentArrow.setXMLName(MBSIMCONTROL%"enableOpenMBVMoment",false);
  }


  Actuator::~Actuator() {
  }

  void Actuator::initialize() {
    Link::initialize();
    inputSignal.initialize();
    connections.initialize();
  }

  DOMElement* Actuator::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    forceDir.initializeUsingXML(element);
    momentDir.initializeUsingXML(element);
    frameOfReference.initializeUsingXML(element);
    inputSignal.initializeUsingXML(element);
    connections.initializeUsingXML(element);
    actuatorForceArrow.initializeUsingXML(element);
    actuatorMomentArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* Actuator::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    forceDir.writeXMLFile(ele0);
    momentDir.writeXMLFile(ele0);
    frameOfReference.writeXMLFile(ele0);
    inputSignal.writeXMLFile(ele0);
    connections.writeXMLFile(ele0);
    actuatorForceArrow.writeXMLFile(ele0);
    actuatorMomentArrow.writeXMLFile(ele0);
    return ele0;
  }

}
