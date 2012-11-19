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
#include "utils.h"
#include <QtGui/QMenu>
#include "frame.h"

using namespace std;


KineticExcitation::KineticExcitation(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind) {

  setText(1,getType());

  properties->addTab("Kinetics");
  //properties->addTab("Constitutive laws");
  properties->addTab("Visualisation");

  forceArrow=new OMBVObjectChoiceWidget("Force arrow",new OMBVArrowWidget,"");
  properties->addToTab("Visualisation",forceArrow);

  momentArrow=new OMBVObjectChoiceWidget("Moment arrow", new OMBVArrowWidget,"");
  properties->addToTab("Visualisation",momentArrow);

  connections = new ConnectWidget("Connections",1,this);
  properties->addToTab("Kinetics",connections);

  force = new ForceLawChoiceWidget("Force", MBSIMNS"force", (OMBVObjectChoiceWidget*)forceArrow);
  properties->addToTab("Kinetics",force);

  moment = new ForceLawChoiceWidget("Moment", MBSIMNS"moment", (OMBVObjectChoiceWidget*)momentArrow);
  properties->addToTab("Kinetics",moment);

  frameOfReference = new FrameOfReferenceWidget("Frame of reference",MBSIMNS"frameOfReference",this,0);
  properties->addToTab("Kinetics",frameOfReference);

  properties->addStretch();
}

KineticExcitation::~KineticExcitation() {
}

void KineticExcitation::initializeUsingXML(TiXmlElement *element) {
  Link::initializeUsingXML(element);
  frameOfReference->initializeUsingXML(element);
  force->initializeUsingXML(element);
  moment->initializeUsingXML(element);
  connections->initializeUsingXML(element);
}

TiXmlElement* KineticExcitation::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  frameOfReference->writeXMLFile(ele0);
  force->writeXMLFile(ele0);
  moment->writeXMLFile(ele0);
  connections->writeXMLFile(ele0);
  return ele0;
}

