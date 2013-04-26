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
#include "signal_.h"
#include "basic_properties.h"
#include "kinetics_properties.h"
#include "basic_widgets.h"
#include "kinetics_widgets.h"
#include "extended_widgets.h"
#include "property.h"

using namespace std;
using namespace MBXMLUtils;

Signal::Signal(const string &str, Element *parent) : Link(str, parent) {
}

Signal::~Signal() {
}

Sensor::Sensor(const string &str, Element *parent) : Signal(str, parent) {
}

Sensor::~Sensor() {
}

AbsoluteCoordinateSensor::AbsoluteCoordinateSensor(const string &str, Element *parent) : Sensor(str, parent) {
  frame.setProperty(new FrameOfReferenceProperty(0,this,MBSIMCONTROLNS"frame"));
  direction.setProperty(new GeneralizedForceDirectionProperty(MBSIMCONTROLNS"direction"));
}

void AbsoluteCoordinateSensor::initialize() {
  Sensor::initialize();
  frame.initialize();
}

void AbsoluteCoordinateSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  frame.initializeUsingXML(element);
  direction.initializeUsingXML(element);
}

TiXmlElement* AbsoluteCoordinateSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  frame.writeXMLFile(ele0);
  direction.writeXMLFile(ele0);
  return ele0;
}

AbsolutePositionSensor::AbsolutePositionSensor(const string &str, Element *parent) : AbsoluteCoordinateSensor(str, parent) {
}
