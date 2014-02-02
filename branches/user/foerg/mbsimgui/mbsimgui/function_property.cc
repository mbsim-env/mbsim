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
#include "function_property.h"
#include "kinematic_functions_properties.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

DOMElement* FunctionProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
  parent->insertBefore(ele0, NULL);
  return ele0;
}

//FunctionChoiceProperty::FunctionChoiceProperty(const string &name) : Property(name), index(0) {
//  //property.push_back(new TranslationAlongXAxisProperty);
//  property = new TranslationAlongXAxisProperty;
//}
//
//void FunctionChoiceProperty::setIndex(int i) {
//  if(index != i) {
//    index = i;
//    delete property;
//    if(i==0)
//      property = new TranslationAlongXAxisProperty;
//    else
//      property = new TranslationAlongYAxisProperty;
//  }
//}
//
//DOMElement* FunctionChoiceProperty::initializeUsingXML(DOMElement *element) {
////  ele1 = E(element)->getFirstElementChildNamed( MBSIM%"mass" );
//  property->initializeUsingXML(element);
//}
//
//DOMElement* FunctionChoiceProperty::writeXMLFile(DOMNode *parent) {
////  DOMDocument *doc=parent->getOwnerDocument();
////  DOMElement *ele1 = D(doc)->createElement( MBSIM%"mass" );
//  property->writeXMLFile(parent);
////  parent->insertBefore(ele1, NULL);
//}


