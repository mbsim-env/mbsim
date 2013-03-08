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
#include "contact.h"
#include "kinetics_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"

using namespace std;

Contact::Contact(const QString &str, QTreeWidgetItem *parentItem, int ind) : Link(str, parentItem, ind) {

  setText(1,getType());

  properties->addTab("Kinetics");
  //properties->addTab("Constitutive laws");
  properties->addTab("Visualisation");

  connections = new ExtWidget("Connections",new ConnectContoursWidget(2,this));
  properties->addToTab("Kinetics", connections);

  contactForceLaw = new ExtWidget("Contact force law",new GeneralizedForceLawChoiceWidget);
  properties->addToTab("Kinetics", contactForceLaw);

  contactImpactLaw = new ExtWidget("Contact impact law",new GeneralizedImpactLawChoiceWidget,true);
  properties->addToTab("Kinetics", contactImpactLaw);

  frictionForceLaw = new ExtWidget("Friction force law",new FrictionForceLawChoiceWidget,true);
  properties->addToTab("Kinetics", frictionForceLaw);

  frictionImpactLaw = new ExtWidget("Friction impact law",new FrictionImpactLawChoiceWidget,true);
  properties->addToTab("Kinetics", frictionImpactLaw);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"),lengthUnits(),4));
  enableOpenMBVContactPoints = new ExtWidget("OpenMBV contact points",new ExtPhysicalVarWidget(input),true); 
  properties->addToTab("Visualisation",enableOpenMBVContactPoints);

  normalForceArrow = new ExtWidget("OpenMBV normal force arrow",new OMBVArrowWidget("NOTSET"),true);
  properties->addToTab("Visualisation",normalForceArrow);

  frictionArrow = new ExtWidget("OpenMBV friction arrow",new OMBVArrowWidget("NOTSET"),true);
  properties->addToTab("Visualisation",frictionArrow);

  properties->addStretch();
}

Contact::~Contact() {
}

void Contact::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Link::initializeUsingXML(element);
  contactForceLaw->initializeUsingXML(element);
  contactImpactLaw->initializeUsingXML(element);
  frictionForceLaw->initializeUsingXML(element);
  frictionImpactLaw->initializeUsingXML(element);
  connections->initializeUsingXML(element);
  enableOpenMBVContactPoints->initializeUsingXML(element);
  normalForceArrow->initializeUsingXML(element);
  frictionArrow->initializeUsingXML(element);
}

TiXmlElement* Contact::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  contactForceLaw->writeXMLFile(ele0);
  contactImpactLaw->writeXMLFile(ele0);
  frictionForceLaw->writeXMLFile(ele0);
  frictionImpactLaw->writeXMLFile(ele0);
  connections->writeXMLFile(ele0);
  enableOpenMBVContactPoints->writeXMLFile(ele0);
  normalForceArrow->writeXMLFile(ele0);
  frictionArrow->writeXMLFile(ele0);
  return ele0;
}
