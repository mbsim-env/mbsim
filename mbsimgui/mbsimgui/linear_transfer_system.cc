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
#include "linear_transfer_system.h"
#include "extended_widgets.h"
#include "variable_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  LinearTransferSystemPropertyDialog::LinearTransferSystemPropertyDialog(LinearTransferSystem *lts, QWidget * parent, Qt::WindowFlags f) : SignalProcessingSystemPropertyDialog(lts,parent,f) {

    choice = new ExtWidget("Type",new ChoiceWidget2(new LinearTransferSystemWidgetFactory));
    addToTab("General", choice);
  }

  LinearTransferSystem::LinearTransferSystem(const QString &str) : SignalProcessingSystem(str) {
  }

  LinearTransferSystemWidgetFactory::LinearTransferSystemWidgetFactory() {
    name.push_back("PID type");
    name.push_back("ABCD type");
    name.push_back("Integrator type");
    name.push_back("PT1 type");
  }

  QWidget* LinearTransferSystemWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;

      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("P",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("I",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("D",new ExtPhysicalVarWidget(input)));

      return widgetContainer;
    }
    if(i==1) {
      ContainerWidget *widgetContainer = new ContainerWidget;

      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("A",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("B",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("C",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("D",new ExtPhysicalVarWidget(input)));

      return widgetContainer;
    }
    if(i==2) {
      ContainerWidget *widgetContainer = new ContainerWidget;

      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("gain",new ExtPhysicalVarWidget(input)));

      return widgetContainer;
    }
    if(i==3) {
      ContainerWidget *widgetContainer = new ContainerWidget;

      vector<PhysicalVariableWidget*> input;
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("P",new ExtPhysicalVarWidget(input)));

      input.clear();
      input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.1"),noUnitUnits(),1));
      widgetContainer->addWidget(new ExtWidget("T",new ExtPhysicalVarWidget(input)));

      return widgetContainer;
    }
    return NULL;
  }

}
