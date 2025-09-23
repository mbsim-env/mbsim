/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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
#include "parameter_context_menu.h"
#include "parameter_view.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "mainwindow.h"
#include "utils.h"

using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterContextMenu::ParameterContextMenu(Parameter *parameter_, QWidget *parent) : QMenu(parent), parameter(parameter_) {
    QAction *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    action->setShortcut(QKeySequence("Ctrl+E"));
    connect(action,&QAction::triggered,this,[=](){ mw->openParameterEditor(); });
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-copy"), "Copy", this);
    action->setShortcut(QKeySequence::Copy);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("edit-cut"), "Cut", this);
    action->setShortcut(QKeySequence::Cut);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(true); });
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    action->setShortcut(QKeySequence("Ctrl+Up"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(true); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)>0);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    action->setShortcut(QKeySequence("Ctrl+Down"));
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(false); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
    action->setShortcut(QKeySequence::Delete);
    connect(action,&QAction::triggered,mw,QOverload<>::of(&MainWindow::removeParameter));
    addAction(action);

    // add the context actions
    addMBSimGUIContextAction(this, parameter);
  }

}
