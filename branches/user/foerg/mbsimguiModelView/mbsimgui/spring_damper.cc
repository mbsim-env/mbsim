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
#include "spring_damper.h"
#include "basic_properties.h"
#include "function_properties.h"
#include "kinetics_properties.h"
#include "ombv_properties.h"
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"

using namespace std;

SpringDamper::SpringDamper(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind), forceDirection(0,false), coilSpring(0,true) {

  setText(1,getType());

  connections.setProperty(new ConnectFramesProperty(2,this));

  forceFunction.setProperty(new Function2ChoiceProperty(MBSIMNS"forceFunction"));

  forceDirection.setProperty(new ForceDirectionProperty(this,MBSIMNS"projectionDirection"));

  coilSpring.setProperty(new OMBVCoilSpringProperty("NOTSET"));
  ((OMBVCoilSpringProperty*)coilSpring.getProperty())->setID(getID());
}

SpringDamper::~SpringDamper() {
}

void SpringDamper::initialize() {
  Link::initialize();
  connections.initialize();
}

void SpringDamper::initializeDialog() {
  Link::initializeDialog();

  dialog->addTab("Kinetics");
  dialog->addTab("Visualisation");

  connectionsWidget = new ExtWidget("Connections",new ConnectFramesWidget(2,this));
  dialog->addToTab("Kinetics", connectionsWidget);

  forceFunctionWidget = new ExtWidget("Force function",new Function2ChoiceWidget);
  dialog->addToTab("Kinetics", forceFunctionWidget);

  forceDirectionWidget = new ExtWidget("Force direction",new ForceDirectionWidget(this),true);
  dialog->addToTab("Kinetics", forceDirectionWidget);

  coilSpringWidget = new ExtWidget("Coil spring",new OMBVCoilSpringWidget("NOTSET"),true);
  dialog->addToTab("Visualisation", coilSpringWidget);
}

void SpringDamper::toWidget() {
  Link::toWidget();
  connections.toWidget(connectionsWidget);
  forceFunction.toWidget(forceFunctionWidget);
  forceDirection.toWidget(forceDirectionWidget);
  coilSpring.toWidget(coilSpringWidget);
}

void SpringDamper::fromWidget() {
  Link::fromWidget();
  connections.fromWidget(connectionsWidget);
  forceFunction.fromWidget(forceFunctionWidget);
  forceDirection.fromWidget(forceDirectionWidget);
  coilSpring.fromWidget(coilSpringWidget);
}

void SpringDamper::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Link::initializeUsingXML(element);
  forceFunction.initializeUsingXML(element);
  forceDirection.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  coilSpring.initializeUsingXML(element);
}

TiXmlElement* SpringDamper::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  forceFunction.writeXMLFile(ele0);
  forceDirection.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  coilSpring.writeXMLFile(ele0);
  return ele0;
}
