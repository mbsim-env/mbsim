/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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
#include "project_property_dialog.h"
#include "project.h"
#include "basic_widgets.h"
#include "extended_widgets.h"
#include <QDialogButtonBox>
#include <QPushButton>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ProjectPropertyDialog::ProjectPropertyDialog(Project *project) : EmbedItemPropertyDialog("Project Properties", project) {
    addTab("General");
    name = new ExtWidget("Name",new TextWidget(project->getName()));
    name->setToolTip("Set the name of the project");
    addToTab("General", name);
    vector<QString> list;
    list.emplace_back("octave");
    list.emplace_back("python");
    list.emplace_back("xmlflat");
    evalSelect = new ExtWidget("Evaluator",new TextChoiceWidget(list,0),true,false,PV%"evaluator");
    addToTab("General",evalSelect);
  }

  DOMElement* ProjectPropertyDialog::initializeUsingXML(DOMElement *parent) {
    static_cast<TextWidget*>(name->getWidget())->setText(item->getName());
    evalSelect->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ProjectPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    E(item->getXMLElement())->setAttribute("name",static_cast<TextWidget*>(name->getWidget())->getText().toStdString());
    evalSelect->writeXMLFile(item->getXMLElement(),item->getXMLElement()->getFirstElementChild());
    static_cast<Project*>(item)->setEvaluator(static_cast<TextChoiceWidget*>(evalSelect->getWidget())->getText().toStdString());
    return nullptr;
  }

}
