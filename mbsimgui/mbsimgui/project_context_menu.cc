/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "project_context_menu.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ProjectContextMenu::ProjectContextMenu(QWidget *parent) : QMenu(parent) {
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openElementEditor(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::editElementSource);
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("dialog-information"), "Add comment", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addCommentToElement(); });
    addAction(action);
  }

}
