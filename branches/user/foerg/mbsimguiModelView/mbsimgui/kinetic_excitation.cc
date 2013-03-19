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
#include "kinetics_properties.h"
#include "kinetics_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"

using namespace std;

KineticExcitation::KineticExcitation(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind), forceArrow(0,true), momentArrow(0,true), force(0,false), moment(0,false), frameOfReference(0,false) {

  setText(1,getType());

  forceArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  ((OMBVArrowProperty*)forceArrow.getProperty())->setID(getID());

  momentArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  ((OMBVArrowProperty*)momentArrow.getProperty())->setID(getID());

  vector<Property*> widget;
  widget.push_back(new ConnectFramesProperty(1,this));
  widget.push_back(new ConnectFramesProperty(2,this));

  connections.setProperty(new PropertyChoiceProperty(widget)); 

  force.setProperty(new ForceChoiceProperty(forceArrow,MBSIMNS"force"));
  moment.setProperty(new ForceChoiceProperty(momentArrow,MBSIMNS"moment"));

  frameOfReference.setProperty(new FrameOfReferenceProperty(0,this,MBSIMNS"frameOfReference"));

}

KineticExcitation::~KineticExcitation() {
}

void KineticExcitation::initialize() {
  Link::initialize();
  connections.initialize();
}

void KineticExcitation::initializeDialog() {
  Link::initializeDialog();

  dialog->addTab("Kinetics");
  dialog->addTab("Visualisation");

  forceArrowWidget = new ExtWidget("OpenMBV force arrow",new OMBVArrowWidget("NOTSET"),true);
  dialog->addToTab("Visualisation",forceArrowWidget);

  momentArrowWidget = new ExtWidget("OpenMBV moment arrow",new OMBVArrowWidget("NOTSET"),true);
  dialog->addToTab("Visualisation",momentArrowWidget);

  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("1 frame");
  name.push_back("2 frames");
  widget.push_back(new ConnectFramesWidget(1,this));
  widget.push_back(new ConnectFramesWidget(2,this));

  connectionsWidget = new ExtWidget("Connections",new WidgetChoiceWidget(name,widget)); 
  dialog->addToTab("Kinetics",connectionsWidget);

  ForceChoiceWidget *f = new ForceChoiceWidget;
  forceWidget = new ExtWidget("Force",f,true);
  dialog->addToTab("Kinetics",forceWidget);

  ForceChoiceWidget *m = new ForceChoiceWidget;
  momentWidget = new ExtWidget("Moment",m,true);
  dialog->addToTab("Kinetics",momentWidget);

  FrameOfReferenceWidget* ref = new FrameOfReferenceWidget(this,0);
  frameOfReferenceWidget = new ExtWidget("Frame of reference",ref,true);
  dialog->addToTab("Kinetics",frameOfReferenceWidget);
}

void KineticExcitation::toWidget() {
  Link::toWidget();
  forceArrow.toWidget(forceArrowWidget);
  momentArrow.toWidget(momentArrowWidget);
  connections.toWidget(connectionsWidget);
  force.toWidget(forceWidget);
  moment.toWidget(momentWidget);
}

void KineticExcitation::fromWidget() {
  Link::fromWidget();
  forceArrow.fromWidget(forceArrowWidget);
  momentArrow.fromWidget(momentArrowWidget);
  connections.fromWidget(connectionsWidget);
  force.fromWidget(forceWidget);
  moment.fromWidget(momentWidget);
}

void KineticExcitation::initializeUsingXML(TiXmlElement *element) {
  Link::initializeUsingXML(element);
  frameOfReference.initializeUsingXML(element);
  force.initializeUsingXML(element);
  moment.initializeUsingXML(element);
  connections.initializeUsingXML(element);
}

TiXmlElement* KineticExcitation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  frameOfReference.writeXMLFile(ele0);
  force.writeXMLFile(ele0);
  moment.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  return ele0;
}

