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

namespace MBSimGUI {

  Contact::Contact(const string &str, Element *parent) : Link(str, parent), contactImpactLaw(0,false), frictionForceLaw(0,false), frictionImpactLaw(0,false), searchAllContactPoints(0,false), initialGuess(0,false), enableOpenMBVContactPoints(0,false), normalForceArrow(0,false), frictionArrow(0,false) {

    connections.setProperty(new ConnectContoursProperty(2,this));

    contactForceLaw.setProperty(new GeneralizedForceLawChoiceProperty(this,MBSIM%"normalForceLaw"));

    contactImpactLaw.setProperty(new GeneralizedImpactLawChoiceProperty(this,MBSIM%"normalImpactLaw"));

    frictionForceLaw.setProperty(new FrictionForceLawChoiceProperty(this,MBSIM%"tangentialForceLaw"));

    frictionImpactLaw.setProperty(new FrictionImpactLawChoiceProperty(this,MBSIM%"tangentialImpactLaw"));

    searchAllContactPoints.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"searchAllContactPoints",vector<string>(2,"")),"",4));

    initialGuess.setProperty(new ChoiceProperty2(new VecPropertyFactory(0,MBSIM%"initialGuess",vector<string>(3,"")),"",4));

    enableOpenMBVContactPoints.setProperty(new FrameMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVContactPoints",getID()));

    normalForceArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVNormalForce",getID()));

    frictionArrow.setProperty(new ArrowMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBVTangentialForce",getID()));
  }

  Contact::~Contact() {
  }

  void Contact::initialize() {
    Link::initialize();
    connections.initialize();
  }

  DOMElement* Contact::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    contactForceLaw.initializeUsingXML(element);
    contactImpactLaw.initializeUsingXML(element);
    frictionForceLaw.initializeUsingXML(element);
    frictionImpactLaw.initializeUsingXML(element);
    connections.initializeUsingXML(element);
    searchAllContactPoints.initializeUsingXML(element);
    initialGuess.initializeUsingXML(element);
    enableOpenMBVContactPoints.initializeUsingXML(element);
    normalForceArrow.initializeUsingXML(element);
    frictionArrow.initializeUsingXML(element);
    return element;
  }

  DOMElement* Contact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    connections.writeXMLFile(ele0);
    contactForceLaw.writeXMLFile(ele0);
    contactImpactLaw.writeXMLFile(ele0);
    frictionForceLaw.writeXMLFile(ele0);
    frictionImpactLaw.writeXMLFile(ele0);
    searchAllContactPoints.writeXMLFile(ele0);
    initialGuess.writeXMLFile(ele0);
    enableOpenMBVContactPoints.writeXMLFile(ele0);
    normalForceArrow.writeXMLFile(ele0);
    frictionArrow.writeXMLFile(ele0);
    return ele0;
  }

}
