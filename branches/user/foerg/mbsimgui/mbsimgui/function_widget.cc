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
#include "function_widget.h"
#include "function_property.h"
#include "mainwindow.h"
#include "objectfactory.h"
#include <QComboBox>
#include <QGridLayout>

extern MainWindow *mw;

FunctionChoiceWidget::FunctionChoiceWidget() {
//  comboBox = new QComboBox;
//  QStringList list;
//  list << "translation x" << "translation y";
//  comboBox->addItems(list);
//  varlayout->addWidget(comboBox);
}

void FunctionChoiceWidget::fromProperty(Property *property) {
//  comboBox->setCurrentIndex(static_cast<FunctionChoiceProperty*>(property)->getIndex());
}

void FunctionChoiceWidget::toProperty(Property *property) {
//  static_cast<FunctionChoiceProperty*>(property)->setIndex(comboBox->currentIndex());
}

FunctionChoiceContextMenu::FunctionChoiceContextMenu(FunctionProperty *property, QWidget *parent, bool removable) : PropertyContextMenu(property,parent,removable) {
  addSeparator();
  FunctionFactory* factory = property->getFactory();
  QActionGroup *actionGroup = new QActionGroup(this);
  for(int i=0; i<factory->size(); i++) {
    QAction *action=new QAction(QString::fromStdString(factory->getName(i)), this);
    action->setCheckable(true);
    actionGroup->addAction(action);
    addAction(action);
    actions[action]=i;
    if(i==4)
      action->setDisabled(true);
    if(property->getName()==factory->getName(i))
      action->setChecked(true);
  }
  connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(setFunction(QAction*)));
}

void FunctionChoiceContextMenu::setFunction(QAction *action) {
  int i = actions[action];

  Property *parent = property->getParent();
  FunctionProperty* function = static_cast<FunctionProperty*>(property)->getFactory()->createFunction(i);
  function->setFactory(static_cast<FunctionProperty*>(property)->getFactory());
  parent->setProperty(function);
//  function->setSignal(parent->sendSignal);
//  if(parent->sendSignal)
//    parent->sendSignal();
  mw->changePropertyItem2(function);
  mw->mbsimxml(1);
}

FunctionChoiceContextMenu2::FunctionChoiceContextMenu2(Property *property, QWidget *parent, bool removable) : PropertyContextMenu(property,parent,removable) {
  addSeparator();
  FunctionFactory1 factory;
  QActionGroup *actionGroup = new QActionGroup(this);
  for(int i=0; i<factory.size(); i++) {
    QAction *action=new QAction(QString::fromStdString("Set "+factory.getName(i)), this);
    action->setCheckable(true);
    actionGroup->addAction(action);
    addAction(action);
    actions[action]=i;
    if(i==4)
      action->setDisabled(true);
    if(property->getProperty()->getName()==factory.getName(i))
      action->setChecked(true);
  }
  connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(setFunction(QAction*)));
}

void FunctionChoiceContextMenu2::setFunction(QAction *action) {
  int i = actions[action];

  FunctionFactory1 factory;
  FunctionProperty* function = factory.createFunction(i);
  property->setProperty(function);
  property->signal();
  mw->changePropertyItem(function);
  mw->mbsimxml(1);
}
