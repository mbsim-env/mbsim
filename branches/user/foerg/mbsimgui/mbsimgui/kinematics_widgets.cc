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
#include "kinematics_widgets.h"
#include "kinematics_properties.h"
#include "mainwindow.h"
#include "objectfactory.h"
#include "function_property.h"

using namespace std;

extern MainWindow *mw;

TranslationChoiceContextMenu::TranslationChoiceContextMenu(Translation *property, QWidget *parent, bool removable) : PropertyContextMenu(property,parent,removable) {
  addSeparator();
  name.push_back("stateDependentTranslation");
  name.push_back("timeDependentTranslation");
  QActionGroup *actionGroup = new QActionGroup(this);
  int index = 0;
  for(int i=0; i<name.size(); i++) {
    QAction *action=new QAction(QString::fromStdString(name[i]), this);
    action->setCheckable(true);
    actionGroup->addAction(action);
    addAction(action);
    actions[action]=i;
    if(property->getName()==name[i]) {
      index = i;
      action->setChecked(true);
    }
  }
  connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(setTranslation(QAction*)));
  addSeparator();
  actionGroup = new QActionGroup(this);
  FunctionFactory* factory;
  if(index==0)
    factory = new FunctionFactory1;
  else
    factory = new FunctionFactory2;
  for(int i=0; i<factory->size(); i++) {
    QAction *action=new QAction(QString::fromStdString(factory->getName(i)), this);
    action->setCheckable(true);
    actionGroup->addAction(action);
    addAction(action);
    actions[action]=i;
    if(property->getProperty()->getName()==factory->getName(i))
      action->setChecked(true);
  }
  delete factory;
  connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(setFunction(QAction*)));
}

void TranslationChoiceContextMenu::setTranslation(QAction *action) {
  int i = actions[action];

  property->setName(name[i]);
  Property* function;
  if(i==0) {
    FunctionFactory1 factory;
    function = factory.createFunction(0);
  }
  else {
    FunctionFactory2 factory;
    function = factory.createFunction(0);
  }
  property->setProperty(function);
  mw->changePropertyItem(function);
  mw->mbsimxml(1);
}

void TranslationChoiceContextMenu::setFunction(QAction *action) {
  int i = actions[action];
  delete property->getProperty();

  FunctionFactory1 factory;
  Property* function = factory.createFunction(i);
  property->setProperty(function);
  mw->changePropertyItem(function);
  mw->mbsimxml(1);
}
