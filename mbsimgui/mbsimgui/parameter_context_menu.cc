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
#include "parameter_context_menu.h"
#include "parameter_view.h"
#include "parameter.h"
#include "embeditemdata.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterContextMenu::ParameterContextMenu(Parameter *parameter_, QWidget *parent) : QMenu(parent), parameter(parameter_) {
    QAction *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openParameterEditor(); });
    QMenu::addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "View XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::viewParameterSource);
    QMenu::addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-copy"), "Copy", this);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("edit-cut"), "Cut", this);
    connect(action,&QAction::triggered,this,[=](){ mw->copyParameter(true); });
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("go-up"), "Move up", this);
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(true); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)>0);
    action=new QAction(QIcon::fromTheme("go-down"), "Move down", this);
    connect(action,&QAction::triggered,this,[=](){ mw->moveParameter(false); });
    addAction(action);
    if(action->isEnabled()) action->setEnabled(parameter->getParent()->getIndexOfParameter(parameter)<parameter->getParent()->getNumberOfParameters()-1);
    addSeparator();
    action=new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
    connect(action,&QAction::triggered,mw,QOverload<>::of(&MainWindow::removeParameter));
    addAction(action);
  }

  void ParameterContextMenu::addAction(QAction *action) {
    if(action->isEnabled()) action->setDisabled((parameter->getParent()->getEmbedItemParent() and parameter->getParent()->getEmbedItemParent()->getEmbeded()) or parameter->getParent()->getEmbededParameters());
    QMenu::addAction(action);
  }

}
