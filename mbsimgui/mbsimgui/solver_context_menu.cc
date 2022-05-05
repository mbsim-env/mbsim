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
#include "solver_context_menu.h"
#include "project.h"
#include "integrator.h"
#include "analyzer.h"
#include "mainwindow.h"
#include "utils.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  SolverContextMenu::SolverContextMenu(QWidget *parent) : QMenu(parent) {
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openElementEditor(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::editElementSource);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-save-as"), "Export", this);
    connect(action,&QAction::triggered,mw,&MainWindow::exportElement);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,&QAction::triggered,this,[=](){ mw->createSolver(mw->loadElement(mw->getProject())); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Solver>(this, nullptr, "Select '");
  }

}
