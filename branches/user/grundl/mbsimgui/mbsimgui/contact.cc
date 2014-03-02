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
using namespace MBXMLUtils;
using namespace xercesc;

Contact::Contact(const string &str, Element *parent) : Link(str, parent), contactImpactLaw(0,false), frictionForceLaw(0,false), frictionImpactLaw(0,false), enableOpenMBVContactPoints(0,false), normalForceArrow(0,false), frictionArrow(0,false) {

  connections.setProperty(new ConnectContoursProperty(2,this));

  contactForceLaw.setProperty(new GeneralizedForceLawChoiceProperty(MBSIM%"normalForceLaw"));

  contactImpactLaw.setProperty(new GeneralizedImpactLawChoiceProperty(""));
  contactImpactLaw.setXMLName(MBSIM%"normalImpactLaw");

  frictionForceLaw.setProperty(new FrictionForceLawChoiceProperty(""));
  frictionForceLaw.setXMLName(MBSIM%"tangentialForceLaw",false);

  frictionImpactLaw.setProperty(new FrictionImpactLawChoiceProperty(""));
  frictionImpactLaw.setXMLName(MBSIM%"tangentialImpactLaw",false);

//  vector<PhysicalVariableProperty> input;
//  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.1"),"m",MBSIM%"enableOpenMBVContactPoints"));
//  enableOpenMBVContactPoints.setProperty(new ExtPhysicalVarProperty(input)); 
  enableOpenMBVContactPoints.setProperty(new OMBVFrameProperty("NOTSET",MBSIM%"enableOpenMBVContactPoints",getID()));

  normalForceArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  normalForceArrow.setXMLName(MBSIM%"enableOpenMBVNormalForce",false);

  frictionArrow.setProperty(new OMBVArrowProperty("NOTSET","",getID()));
  frictionArrow.setXMLName(MBSIM%"enableOpenMBVTangentialForce",false);
}

Contact::~Contact() {
}

void Contact::initialize() {
  Link::initialize();
  connections.initialize();
}

void Contact::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
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

DOMElement* Contact::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = Link::writeXMLFile(parent);
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
