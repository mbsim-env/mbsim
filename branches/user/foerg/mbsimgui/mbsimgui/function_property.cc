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
#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbxmlutilstinyxml/tinynamespace.h>

using namespace std;
using namespace MBXMLUtils;

TiXmlElement* FunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
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
//TiXmlElement* FunctionChoiceProperty::initializeUsingXML(TiXmlElement *element) {
////  ele1 = element->FirstChildElement( MBSIMNS"mass" );
//  property->initializeUsingXML(element);
//}
//
//TiXmlElement* FunctionChoiceProperty::writeXMLFile(TiXmlNode *parent) {
////  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"mass" );
//  property->writeXMLFile(parent);
////  parent->LinkEndChild(ele1);
//}


