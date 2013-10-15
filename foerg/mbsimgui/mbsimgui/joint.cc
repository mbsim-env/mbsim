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
#include "kinetics_properties.h"
#include "ombv_properties.h"

using namespace std;
using namespace MBXMLUtils;

Joint::Joint(const string &str, Element *parent) : Link(str, parent), refFrameID(0,false), forceDirection(0,false), forceLaw(0,false), momentDirection(0,false), momentLaw(0,false), forceArrow(0,false), momentArrow(0,false) {

  refFrameID.setProperty(new IntegerProperty(0,MBSIMNS"frameOfReferenceID"));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"forceDirection"));
  forceDirection.setProperty(new ExtPhysicalVarProperty(input));

  forceLaw.setProperty(new GeneralizedForceLawChoiceProperty(MBSIMNS"forceLaw"));

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"momentDirection"));
  momentDirection.setProperty(new ExtPhysicalVarProperty(input));

  momentLaw.setProperty(new GeneralizedForceLawChoiceProperty(MBSIMNS"momentLaw"));

  connections.setProperty(new ConnectFramesProperty(2,this));

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  forceArrow.setXMLName(MBSIMNS"openMBVForceArrow",false);

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET",getID()));
  momentArrow.setXMLName(MBSIMNS"openMBVMomentArrow",false);
}

Joint::~Joint() {
}

void Joint::initialize() {
  Link::initialize();
  connections.initialize();
}

void Joint::initializeUsingXML(TiXmlElement *element) {
  Link::initializeUsingXML(element);
  refFrameID.initializeUsingXML(element);
  forceDirection.initializeUsingXML(element);
  forceLaw.initializeUsingXML(element);
  momentDirection.initializeUsingXML(element);
  momentLaw.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  forceArrow.initializeUsingXML(element);
  momentArrow.initializeUsingXML(element);
}

TiXmlElement* Joint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  refFrameID.writeXMLFile(ele0);
  forceDirection.writeXMLFile(ele0);
  forceLaw.writeXMLFile(ele0);
  momentDirection.writeXMLFile(ele0);
  momentLaw.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  forceArrow.writeXMLFile(ele0);
  momentArrow.writeXMLFile(ele0);
  return ele0;
}
