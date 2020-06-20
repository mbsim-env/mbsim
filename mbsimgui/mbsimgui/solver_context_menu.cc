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
#include "solver_context_menu.h"
#include "integrator.h"
#include "analyzer.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  SolverContextMenu::SolverContextMenu(QWidget *parent) : QMenu(parent) {
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openSolverEditor(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "View XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::viewSolverSource);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-save-as"), "Export", this);
    connect(action,&QAction::triggered,mw,&MainWindow::exportSolver);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Embed", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadSolver(true); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("document-open"), "Import", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadSolver(); });
    addAction(action);
    addSeparator();
    QMenu *analyzers = new QMenu("Select analyzer", this);
    action = new QAction("Select eigenanalyzer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new Eigenanalyzer); });
    analyzers->addAction(action);
    action = new QAction("Select harmonic response analyzer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new HarmonicResponseAnalyzer); });
    analyzers->addAction(action);
    addMenu(analyzers);
    QMenu *integrators = new QMenu("Select integrator", this);
    action = new QAction("Select DOPRI5 Integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new DOPRI5Integrator); });
    integrators->addAction(action);
    addMenu(integrators);
  }

}
