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

StateDependentTranslation::StateDependentTranslation(const string &name) : Property(name) {
  FunctionFactory1 factory;
  addProperty(factory.createFunction(0));
}

TiXmlElement* StateDependentTranslation::initializeUsingXML(TiXmlElement *element) {
  property[0]->initializeUsingXML(element);
}

TiXmlElement* StateDependentTranslation::writeXMLFile(TiXmlNode *parent) {
  property[0]->writeXMLFile(parent);
}

Translation::Translation(const std::string &name) : Property(name) { 
 // addProperty(new StateDependentTranslation("stateDependentTranslation"));
 // property[0]->setDisabling(true);
 // property[0]->setDisabled(true);
  FunctionFactory1 factory;
  addProperty(factory.createFunction(0));
}

TiXmlElement* Translation::initializeUsingXML(TiXmlElement *parent) {
  return property[0]->initializeUsingXML(parent);
}

TiXmlElement* Translation::writeXMLFile(TiXmlNode *parent) {
  return property[0]->writeXMLFile(parent);
}

//TiXmlElement* Translation::initializeUsingXML(TiXmlElement *parent) {
//  index = -1;
//  for(int i=0; i<property.size(); i++) {
//    if(property[i]->initializeUsingXML(parent)) {
//      index = i;
//      break;
//    }
//  }
//  if(index == -1) {
//    cout << "Mist" << endl;
//    throw;
//  }
//}
//
//TiXmlElement* Translation::writeXMLFile(TiXmlNode *parent) {
//  return property[index]->writeXMLFile(parent);
//}
//
