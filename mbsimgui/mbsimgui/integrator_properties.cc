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
using namespace xercesc;

namespace MBSimGUI {

  RKSuiteTypeProperty::RKSuiteTypeProperty() : index(0) {
    method.push_back(MBSIMINT%"method23");
    method.push_back(MBSIMINT%"method45");
    method.push_back(MBSIMINT%"method67");
  }

  void RKSuiteTypeProperty::fromWidget(QWidget *widget) {
    index = static_cast<RKSuiteTypeWidget*>(widget)->comboBox->currentIndex();
  }

  void RKSuiteTypeProperty::toWidget(QWidget *widget) {
    static_cast<RKSuiteTypeWidget*>(widget)->comboBox->setCurrentIndex(index);
  }

  DOMElement* RKSuiteTypeProperty::initializeUsingXML(DOMElement *element) {}

  DOMElement* RKSuiteTypeProperty::writeXMLFile(DOMNode *element) {
    DOMDocument *doc=element->getOwnerDocument();
    DOMElement *ele = D(doc)->createElement(method[index]);
    element->insertBefore(ele, NULL);
    return 0;
  }

}
