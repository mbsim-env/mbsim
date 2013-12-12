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
#include "property.h"
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;
using namespace MBXMLUtils;

TiXmlElement* Property::initializeUsingXML(TiXmlElement *element) {
  for(int i=0; i<property.size(); i++)
    property[i]->initializeUsingXML(element);
}

TiXmlElement* Property::writeXMLFile(TiXmlNode *element) {
  for(int i=0; i<property.size(); i++)
    property[i]->writeXMLFile(element);
}

void PhysicalProperty::setValue(const string &data) { 
  Property::setValue(data); 
  try {
  setEvaluation(OctEval::cast<string>(MainWindow::octEval->stringToOctValue(getValue())));
  }
  catch(...) {
    cout << "an execption was thrown" << endl;
  }
}

TiXmlElement* PhysicalProperty::initializeUsingXML(TiXmlElement *element) {
  if(element->Attribute("unit"))
    setCurrentUnit(units.find(element->Attribute("unit")));
}

TiXmlElement* PhysicalProperty::writeXMLFile(TiXmlNode *parent) {
  if(units.getNumberOfUnits()) {
    TiXmlElement *ele = (TiXmlElement*)parent;
    ele->SetAttribute("unit", getUnit());
  }
}

