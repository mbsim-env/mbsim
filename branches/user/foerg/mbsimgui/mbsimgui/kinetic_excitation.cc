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
//#include "kinetics_properties.h"
#include "function_properties.h"

using namespace std;
using namespace MBXMLUtils;

KineticExcitation::KineticExcitation(const string &str, Element *parent) : Link(str, parent), forceArrow(0,true), momentArrow(0,true), forceDirection(0,false), forceFunction(0,false), momentDirection(0,false), momentFunction(0,false), frameOfReference(0,false) {

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  forceArrow.setXMLName(MBSIMNS"openMBVForceArrow",false);

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  momentArrow.setXMLName(MBSIMNS"openMBVMomentArrow",false);

  vector<Property*> widget;
  widget.push_back(new ConnectFramesProperty(1,this));
  widget.push_back(new ConnectFramesProperty(2,this));

  connections.setProperty(new ChoiceProperty("",widget,2)); 

  //force.setProperty(new ForceChoiceProperty(forceArrow,MBSIMNS"force"));
  //moment.setProperty(new ForceChoiceProperty(momentArrow,MBSIMNS"moment"));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"forceDirection"));
  forceDirection.setProperty(new ExtPhysicalVarProperty(input));

  vector<Property*> property;
  property.push_back(new ConstantFunctionProperty("V"));
  property.push_back(new LinearFunctionProperty("V"));
  property.push_back(new QuadraticFunctionProperty("V"));
  property.push_back(new SinusFunctionProperty("V"));
  property.push_back(new TabularFunctionProperty("V"));
  property.push_back(new LinearCombinationFunctionProperty("V"));
  property.push_back(new PiecewiseDefinedFunctionProperty("V"));
  vector<string> var;
  var.push_back("t");
  property.push_back(new SymbolicFunctionProperty("VS",var));
  forceFunction.setProperty(new ChoiceProperty(MBSIMNS"forceFunction",property));

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"momentDirection"));
  momentDirection.setProperty(new ExtPhysicalVarProperty(input));

  property.clear();
  property.push_back(new ConstantFunctionProperty("V"));
  property.push_back(new LinearFunctionProperty("V"));
  property.push_back(new QuadraticFunctionProperty("V"));
  property.push_back(new SinusFunctionProperty("V"));
  property.push_back(new TabularFunctionProperty("V"));
  property.push_back(new LinearCombinationFunctionProperty("V"));
  property.push_back(new PiecewiseDefinedFunctionProperty("V"));
  var.clear();
  var.push_back("t");
  property.push_back(new SymbolicFunctionProperty("VS",var));
  momentFunction.setProperty(new ChoiceProperty(MBSIMNS"momentFunction",property));

  frameOfReference.setProperty(new FrameOfReferenceProperty("",this,MBSIMNS"frameOfReference"));

}

void KineticExcitation::initialize() {
  Link::initialize();
  connections.initialize();
}

void KineticExcitation::initializeUsingXML(TiXmlElement *element) {
  Link::initializeUsingXML(element);
  frameOfReference.initializeUsingXML(element);
  forceDirection.initializeUsingXML(element);
  forceFunction.initializeUsingXML(element);
  momentDirection.initializeUsingXML(element);
  momentFunction.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
  momentArrow.initializeUsingXML(element);
}

TiXmlElement* KineticExcitation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  frameOfReference.writeXMLFile(ele0);
  forceDirection.writeXMLFile(ele0);
  forceFunction.writeXMLFile(ele0);
  momentDirection.writeXMLFile(ele0);
  momentFunction.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  momentArrow.writeXMLFile(ele0);
  return ele0;
}

