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
#include "basic_widgets.h"
#include "kinetics_widgets.h"
#include "extended_widgets.h"

using namespace std;


Signal::Signal(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind) {
  ns = MBSIMCONTROLNS;
}

Signal::~Signal() {
}

Sensor::Sensor(const QString &str, QTreeWidgetItem *parentItem, int ind) : Signal(str, parentItem, ind) {
}

Sensor::~Sensor() {
}

AbsolutCoordinateSensor::AbsolutCoordinateSensor(const QString &str, QTreeWidgetItem *parentItem, int ind) : Sensor(str, parentItem, ind) {
  frame = new ExtXMLWidget("Frame of reference",new FrameOfReferenceWidget(MBSIMCONTROLNS"frame",this,0));
  properties->addToTab("General", frame);
  direction = new ExtXMLWidget("Direction",new GeneralizedForceDirectionWidget(MBSIMCONTROLNS"direction"));
  properties->addToTab("General", direction);
}

void AbsolutCoordinateSensor::initializeUsingXML(TiXmlElement *element) {
  Sensor::initializeUsingXML(element);
  frame->initializeUsingXML(element);
  direction->initializeUsingXML(element);
}

TiXmlElement* AbsolutCoordinateSensor::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Sensor::writeXMLFile(parent);
  frame->writeXMLFile(ele0);
  direction->writeXMLFile(ele0);
  return ele0;
}

AbsolutePositionSensor::AbsolutePositionSensor(const QString &str, QTreeWidgetItem *parentItem, int ind) : AbsolutCoordinateSensor(str, parentItem, ind) {
  setText(1,getType());
  properties->addStretch();
}
