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
#include "integrator_properties.h"
#include "integrator_widgets.h"

using namespace MBXMLUtils;

RKSuiteTypeProperty::RKSuiteTypeProperty() : index(0) {
  method.push_back(MBSIMINTNS"method23");
  method.push_back(MBSIMINTNS"method45");
  method.push_back(MBSIMINTNS"method67");
}

void RKSuiteTypeProperty::fromWidget(QWidget *widget) {
  index = static_cast<RKSuiteTypeWidget*>(widget)->comboBox->currentIndex();
}

void RKSuiteTypeProperty::toWidget(QWidget *widget) {
  static_cast<RKSuiteTypeWidget*>(widget)->comboBox->setCurrentIndex(index);
}

TiXmlElement* RKSuiteTypeProperty::initializeUsingXML(TiXmlElement *element) {}

TiXmlElement* RKSuiteTypeProperty::writeXMLFile(TiXmlNode *element) {
  TiXmlElement *ele = new TiXmlElement(method[index]);
  element->LinkEndChild(ele);
  return 0;
}
