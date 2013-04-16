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
#include "basic_properties.h"
#include "function_properties.h"
#include "kinetics_properties.h"
#include "ombv_properties.h"

using namespace std;

Contact::Contact(const string &str, Element *parent) : Link(str, parent), contactImpactLaw(0,false), frictionForceLaw(0,false), frictionImpactLaw(0,false), enableOpenMBVContactPoints(0,false), normalForceArrow(0,false), frictionArrow(0,false) {

  connections.setProperty(new ConnectContoursProperty(2,this));

  contactForceLaw.setProperty(new GeneralizedForceLawChoiceProperty(MBSIMNS"contactForceLaw"));

  contactImpactLaw.setProperty(new GeneralizedImpactLawChoiceProperty(""));
  contactImpactLaw.setXMLName(MBSIMNS"contactImpactLaw");

  frictionForceLaw.setProperty(new FrictionForceLawChoiceProperty(""));
  frictionForceLaw.setXMLName(MBSIMNS"frictionForceLaw");

  frictionImpactLaw.setProperty(new FrictionImpactLawChoiceProperty(""));
  frictionImpactLaw.setXMLName(MBSIMNS"frictionImpactLaw");

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0.1"),"m",MBSIMNS"enableOpenMBVContactPoints"));
  enableOpenMBVContactPoints.setProperty(new ExtPhysicalVarProperty(input)); 

  normalForceArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  normalForceArrow.setXMLName(MBSIMNS"openMBVNormalForceArrow",false);

  frictionArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  frictionArrow.setXMLName(MBSIMNS"openMBVFrictionArrow",false);
}

Contact::~Contact() {
}

void Contact::initialize() {
  Link::initialize();
  connections.initialize();
}

void Contact::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Link::initializeUsingXML(element);
  contactForceLaw.initializeUsingXML(element);
  contactImpactLaw.initializeUsingXML(element);
  frictionForceLaw.initializeUsingXML(element);
  frictionImpactLaw.initializeUsingXML(element);
  connections.initializeUsingXML(element);
  enableOpenMBVContactPoints.initializeUsingXML(element);
  normalForceArrow.initializeUsingXML(element);
  frictionArrow.initializeUsingXML(element);
}

TiXmlElement* Contact::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Link::writeXMLFile(parent);
  contactForceLaw.writeXMLFile(ele0);
  contactImpactLaw.writeXMLFile(ele0);
  frictionForceLaw.writeXMLFile(ele0);
  frictionImpactLaw.writeXMLFile(ele0);
  connections.writeXMLFile(ele0);
  enableOpenMBVContactPoints.writeXMLFile(ele0);
  normalForceArrow.writeXMLFile(ele0);
  frictionArrow.writeXMLFile(ele0);
  return ele0;
}
