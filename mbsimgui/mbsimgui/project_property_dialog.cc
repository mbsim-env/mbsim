/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

  ProjectPropertyDialog::ProjectPropertyDialog(Project *project, QWidget *parent, const Qt::WindowFlags& f) : EmbedItemPropertyDialog(project,parent,f) {
    addTab("General");
    name = new ExtWidget("Name",new TextWidget(project->getName()));
    name->setToolTip("Set the name of the project");
    addToTab("General", name);
    vector<QString> list;
    list.emplace_back("octave");
    list.emplace_back("python");
    evalSelect = new ExtWidget("Evaluator",new TextChoiceWidget(list,0),true,false,PV%"evaluator");
    evalSelect->setDisabled(true);
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
    return nullptr;
  }

}
