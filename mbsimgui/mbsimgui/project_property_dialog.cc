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
    for(auto &x : Evaluator::evaluators)
      list.emplace_back(x.c_str());
    evalSelect = new ExtWidget("Evaluator",new TextChoiceWidget(list,project->getDefaultEvaluator()),true,false,PV%"evaluator");

    // changing the evaluator is quite tricky:
    // - if the model already contains evaluator code this will fail with a changed evaluator
    // - switching the evaluator is complicated (or partially impossible) regarding DLL/so loading inside the same process
    // -> do not allow to change the evaluator in mbsimgui
    evalSelect->setDisabled(true);

    addToTab("General",evalSelect);
    addTab("Comment");
    comment = new CommentWidget;
    addToTab("Comment", comment);
  }

  DOMElement* ProjectPropertyDialog::initializeUsingXML(DOMElement *parent) {
    name->getWidget<TextWidget>()->setText(item->getName());
    comment->initializeUsingXML(item->getXMLElement());
    evalSelect->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ProjectPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    E(item->getXMLElement())->setAttribute("name",name->getWidget<TextWidget>()->getText().toStdString());
    comment->writeXMLFile(item->getXMLElement(),ref);
    item->updateName();
    evalSelect->writeXMLFile(item->getXMLElement(),item->getXMLElement()->getFirstElementChild());

    // we do not need to check here for changed evaluator since we disallow changing the evaluator in the Project
    // property dialog, see above.
    if(evalSelect->isActive())
      static_cast<Project*>(item)->setEvaluator(Eval::createEvaluator(evalSelect->getWidget<TextChoiceWidget>()->getText().toStdString()));
    else
      static_cast<Project*>(item)->setEvaluator(Eval::createEvaluator(Evaluator::defaultEvaluator));
    return nullptr;
  }

}
