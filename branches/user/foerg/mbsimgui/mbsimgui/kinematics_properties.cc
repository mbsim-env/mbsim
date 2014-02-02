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
#include "function_property_factory.h"
#include "kinematics_properties.h"
#include "kinematic_functions_properties.h"
#include "objectfactory.h"
  
using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

Translation::Translation(const std::string &name) : Property(name) { 
  FunctionFactory1 *factory = new FunctionFactory1;
  FunctionProperty *function = factory->createFunction(0);
  delete factory;
  addProperty(function);
}

DOMElement* Translation::initializeUsingXML(DOMElement *parent) {
  FunctionFactory *factory;
  if(name=="stateDependentTranslation")
    factory = new FunctionFactory1;
  else
    factory = new FunctionFactory2;
  FunctionProperty *function = factory->createFunction(parent->getFirstElementChild());
  delete factory;
  setProperty(function);
  return property[0]->initializeUsingXML(parent->getFirstElementChild());
}

DOMElement* Translation::writeXMLFile(DOMNode *parent) {
  return property[0]->writeXMLFile(parent);
}

int Translation::getqSize() const {
  return name=="stateDependentTranslation"?static_cast<FunctionProperty*>(property[0])->getArgSize():0;
}

int Translation::getuSize() const {
  return name=="stateDependentTranslation"?static_cast<FunctionProperty*>(property[0])->getArgSize():0;
}
