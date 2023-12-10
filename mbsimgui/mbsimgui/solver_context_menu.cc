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
    setToolTipsVisible(true);
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openElementEditor(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "Edit XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::editElementSource);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-save-as"), "Export solver to file...", this);
    connect(action,&QAction::triggered,[](){ mw->exportElement("Export Solver"); } );
    addAction(action);
    addSeparator();
    action=new QAction(QIcon::fromTheme("dialog-information"), "Add comment", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addCommentToElement(); });
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Import/Reference solver from file (replacing the Solver) ...", this);
    action->setToolTip("Import a solver from a file or use the XML 'Embed' functionality to reference a external solver file.");
    connect(action,&QAction::triggered,this,[=](){ mw->createSolver(mw->loadEmbedItemData(mw->getProject(), "Import/Reference Solver")); });
    addAction(action);
    addSeparator();
    createContextMenuFor<Solver>(this, nullptr, "Select '");
  }

}
