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
#include "kinetic_excitation.h"
#include "ombv_properties.h"
#include "function_properties.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

KineticExcitation::KineticExcitation(const string &str, Element *parent) : Link(str, parent), refFrameID(0,false), forceDirection(0,false), forceFunction(0,false), momentDirection(0,false), momentFunction(0,false), forceArrow(0,true), momentArrow(0,true) {

  refFrameID.setProperty(new IntegerProperty(1,MBSIM%"frameOfReferenceID"));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"forceDirection"));
  forceDirection.setProperty(new ExtPhysicalVarProperty(input));

  forceFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIM%"forceFunction",0));

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIM%"momentDirection"));
  momentDirection.setProperty(new ExtPhysicalVarProperty(input));

  momentFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIM%"momentFunction",0));

 // vector<Property*> widget;
 // widget.push_back(new ConnectFramesProperty(1,this));
 // widget.push_back(new ConnectFramesProperty(2,this));

 // connections.setProperty(new ChoiceProperty2("",widget,2)); 
  connections.setProperty(new ChoiceProperty2(new ConnectFramesPropertyFactory(this),"",4)); 

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  forceArrow.setXMLName(MBSIM%"openMBVForceArrow",false);

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  momentArrow.setXMLName(MBSIM%"openMBVMomentArrow",false);
}

void KineticExcitation::initialize() {
  Link::initialize();
  connections.initialize();
}

void KineticExcitation::initializeUsingXML(DOMElement *element) {
  Link::initializeUsingXML(element);
  refFrameID.initializeUsingXML(element);
  forceDirection.initializeUsingXML(element);
  forceFunction.initializeUsingXML(element);
  momentDirection.initializeUsingXML(element);
  momentFunction.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
  momentArrow.initializeUsingXML(element);
}

DOMElement* KineticExcitation::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Link::writeXMLFile(parent);
  refFrameID.writeXMLFile(ele0);
  forceDirection.writeXMLFile(ele0);
  forceFunction.writeXMLFile(ele0);
  momentDirection.writeXMLFile(ele0);
  momentFunction.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  momentArrow.writeXMLFile(ele0);
  return ele0;
}

