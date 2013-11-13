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
#include "property_context_menu.h"
#include "function_property.h"
#include "mainwindow.h"

extern MainWindow *mw;

PropertyContextMenu::PropertyContextMenu(Property *property_, QWidget *parent, bool removable) : QMenu(parent), property(property_) {
  QAction *action=new QAction("Edit", this);
  connect(action,SIGNAL(triggered()),mw,SLOT(openPropertyDialog()));
  addAction(action);
  if(property->disabling()) {
    addSeparator();
    QAction *action=new QAction("Disabled", this);
    action->setCheckable(true);
    action->setChecked(property->isDisabled());
    connect(action,SIGNAL(toggled(bool)),this,SLOT(disableProperty(bool)));
    addAction(action);
  }
  if(removable) {
    addSeparator();
    QAction *action=new QAction("Remove", this);
//    connect(action,SIGNAL(triggered()),mw,SLOT(removeElement()));
    addAction(action);
    addSeparator();
  }
}

void PropertyContextMenu::disableProperty(bool disable) {
  property->setDisabled(disable);
  mw->mbsimxml(1);
}

ChoiceProperty2ContextMenu::ChoiceProperty2ContextMenu(Property *property, QWidget *parent, bool removable) : PropertyContextMenu(property,parent,removable) {
  addSeparator();
  PropertyFactory *factory = static_cast<ChoiceProperty2*>(property)->getPropertyFactory();
  for(int i=0; i<factory->getSize(); i++) {
    QAction *action=new QAction(QString::fromStdString(factory->getName(i)), this);
    connect(action,SIGNAL(triggered()),this,SLOT(selectProperty()));
    addAction(action);
  }
}

void ChoiceProperty2ContextMenu::selectProperty() {
}

//FunctionChoiceContextMenu::FunctionChoiceContextMenu(Property *property, QWidget *parent, bool removable) : PropertyContextMenu(property,parent,removable) {
//  addSeparator();
//  QStringList list;
//  list << "translation x" << "translation y";
//  QActionGroup *actionGroup = new QActionGroup(this);
//  for(int i=0; i<list.size(); i++) {
//    QAction *action=new QAction(list[i], this);
//    action->setCheckable(true);
//    actionGroup->addAction(action);
//    addAction(action);
//    actions[action]=i;
//    if(i==static_cast<FunctionChoiceProperty*>(property)->getIndex())
//      action->setChecked(true);
//  }
//  connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(setFunction(QAction*)));
//}
//
//void FunctionChoiceContextMenu::setFunction(QAction *action) {
//  static_cast<FunctionChoiceProperty*>(property)->setIndex(actions[action]);
//  mw->mbsimxml(1);
//}
