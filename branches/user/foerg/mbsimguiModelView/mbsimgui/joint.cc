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
#include "kinetics_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"

using namespace std;

Joint::Joint(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind), forceArrow(0,false), momentArrow(0,false), force(0,false), moment(0,false) {

  setText(1,getType());

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  ((OMBVArrowProperty*)forceArrow.getProperty())->setID(getID());

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  ((OMBVArrowProperty*)momentArrow.getProperty())->setID(getID());

  connections.setProperty(new ConnectFramesProperty(2,this));

  force.setProperty(new GeneralizedForceChoiceProperty(forceArrow,MBSIMNS"force"));

  moment.setProperty(new GeneralizedForceChoiceProperty(momentArrow,MBSIMNS"moment"));
}

Joint::~Joint() {
}

void Joint::initialize() {
  Link::initialize();
  connections.initialize();
}

void Joint::initializeDialog() {
  Link::initializeDialog();

  dialog->addTab("Kinetics");
  dialog->addTab("Visualisation");

  forceArrowWidget = new ExtWidget("OpenMBV force arrow",new OMBVArrowWidget("NOTSET"),true);
  ((OMBVArrowWidget*)forceArrowWidget->getWidget())->setID(getID());
  dialog->addToTab("Visualisation",forceArrowWidget);

  momentArrowWidget = new ExtWidget("OpenMBV moment arrow",new OMBVArrowWidget("NOTSET"),true);
  ((OMBVArrowWidget*)momentArrowWidget->getWidget())->setID(getID());
  dialog->addToTab("Visualisation",momentArrowWidget);

  connectionsWidget = new ExtWidget("Connections",new ConnectFramesWidget(2,this));
  dialog->addToTab("Kinetics", connectionsWidget);

  forceWidget = new ExtWidget("Force",new GeneralizedForceChoiceWidget,true);
  dialog->addToTab("Kinetics", forceWidget);

  momentWidget = new ExtWidget("Moment",new GeneralizedForceChoiceWidget,true);
  dialog->addToTab("Kinetics", momentWidget);
}

void Joint::toWidget() {
  Link::toWidget();
  forceArrow.toWidget(forceArrowWidget);
  momentArrow.toWidget(momentArrowWidget);
  connections.toWidget(connectionsWidget);
  force.toWidget(forceWidget);
  moment.toWidget(momentWidget);
}

void Joint::fromWidget() {
  Link::fromWidget();
  forceArrow.fromWidget(forceArrowWidget);
  momentArrow.fromWidget(momentArrowWidget);
  connections.fromWidget(connectionsWidget);
  force.fromWidget(forceWidget);
  moment.fromWidget(momentWidget);
}

void Joint::initializeUsingXML(TiXmlElement *element) {
  Link::initializeUsingXML(element);
  force.initializeUsingXML(element);
 // moment.initializeUsingXML(element);
  connections.initializeUsingXML(element);
}

TiXmlElement* Joint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  force.writeXMLFile(ele0);
 // moment.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  return ele0;
}
