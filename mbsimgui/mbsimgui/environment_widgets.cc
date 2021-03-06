/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

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
#include "environment_widgets.h"
#include "extended_widgets.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "unknown_widget.h"
#include <QVBoxLayout>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* EnvironmentWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  MBSimEnvironmentWidget::MBSimEnvironmentWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    accelerationOfGravity = new ExtWidget("Acceleration of gravity",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,accelerationUnits())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"accelerationOfGravity");
    layout->addWidget(accelerationOfGravity);

    openMBVObject = new ExtWidget("OpenMBV object",new OpenMBVEnvironmentWidget,true,false,MBSIM%"openMBVObject");
    layout->addWidget(openMBVObject);
  }

  DOMElement* MBSimEnvironmentWidget::initializeUsingXML(DOMElement *element) {
    accelerationOfGravity->initializeUsingXML(element);
    openMBVObject->initializeUsingXML(element);
    return element;
  }

  DOMElement* MBSimEnvironmentWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = EnvironmentWidget::writeXMLFile(parent);
    accelerationOfGravity->writeXMLFile(ele0);
    openMBVObject->writeXMLFile(ele0);
    return ele0;
  }

  EnvironmentWidgetFactory::EnvironmentWidgetFactory() {
    name.emplace_back("MBSim environment");
    name.emplace_back("Unknown environment");
    xmlName.push_back(MBSIM%"MBSimEnvironment");
    xmlName.push_back(MBSIM%"UnknownEnvironment");
  }

  Widget* EnvironmentWidgetFactory::createWidget(int i) {
    if(i==0)
      return new MBSimEnvironmentWidget;
    if(i==1)
      return new UnknownWidget;
    return nullptr;
  }
}
