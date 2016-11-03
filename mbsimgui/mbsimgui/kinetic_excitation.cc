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
#include "function_properties.h"
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  KineticExcitation::KineticExcitation(const string &str, Element *parent) : FloatingFrameLink(str, parent), forceDirection(0,false), forceFunction(0,false), momentDirection(0,false), momentFunction(0,false) {

    forceDirection.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIM%"forceDirection",vector<string>(3,"-")),"",4));

    forceFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"forceFunction",0));

    momentDirection.setProperty(new ChoiceProperty2(new MatPropertyFactory(getMat<string>(3,1,"0"),MBSIM%"momentDirection",vector<string>(3,"-")),"",4));

    momentFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"momentFunction",0));
  }

//  void KineticExcitation::initialize() {
//    FloatingFrameLink::initialize();
//    forceFunction.initialize();
//  }

  DOMElement* KineticExcitation::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    forceDirection.initializeUsingXML(element);
    forceFunction.initializeUsingXML(element);
    momentDirection.initializeUsingXML(element);
    momentFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* KineticExcitation::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = FloatingFrameLink::writeXMLFile(parent);
    forceDirection.writeXMLFile(ele0);
    forceFunction.writeXMLFile(ele0);
    momentDirection.writeXMLFile(ele0);
    momentFunction.writeXMLFile(ele0);
    return ele0;
  }

}
