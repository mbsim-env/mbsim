/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2020 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "environment_widgets.h"
#include "extended_widgets.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "unknown_widget.h"
#include "objectfactory.h"
#include <QVBoxLayout>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(MBSimEnvironmentWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<EnvironmentWidget>);

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

    openMBVObject = new ExtWidget("OpenMBV objects",new ListWidget(new ChoiceWidgetFactory(new OMBVRigidBodyWidgetFactory,1),"OpenMBV object",1,0,false),true,false,MBSIM%"openMBVObject");
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

}
