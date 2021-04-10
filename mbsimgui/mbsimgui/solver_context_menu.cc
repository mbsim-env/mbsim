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
    connect(action,&QAction::triggered,this,[=](){ mw->loadSolver(); });
    addAction(action);
    addSeparator();
    QMenu *analyzers = new QMenu("Select analyzer", this);
    action = new QAction("Select linear system analyzer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new LinearSystemAnalyzer); });
    analyzers->addAction(action);
    addMenu(analyzers);
    QMenu *integrators = new QMenu("Select integrator", this);
    action = new QAction("Select Boost odeint DOS RKDOPRI5", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new BoostOdeintDOS_RKDOPRI5); });
    integrators->addAction(action);
    action = new QAction("Select Boost odeint DOS Burlisch Stoer", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new BoostOdeintDOS_BulirschStoer); });
    integrators->addAction(action);
    action = new QAction("Select Boost odeint DOS Rosenbrock4", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new BoostOdeintDOS_Rosenbrock4); });
    integrators->addAction(action);
    action = new QAction("Select DASPK integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new DASPKIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select DOPRI5 Integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new DOPRI5Integrator); });
    integrators->addAction(action);
    action = new QAction("Select DOP853 integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new DOP853Integrator); });
    integrators->addAction(action);
    action = new QAction("Select Explicit Euler integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new ExplicitEulerIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select HETS2 integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new HETS2Integrator); });
    integrators->addAction(action);
    action = new QAction("Select Implicit Euler integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new ImplicitEulerIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select LSODA integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new LSODAIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select LSODE integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new LSODEIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select LSODI integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new LSODIIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select ODEX integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new ODEXIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select Quasi static integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new QuasiStaticIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select PHEM56 integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new PHEM56Integrator); });
    integrators->addAction(action);
    action = new QAction("Select RADAU integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new RADAUIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select RADAU5 integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new RADAU5Integrator); });
    integrators->addAction(action);
    action = new QAction("Select RKSuite integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new RKSuiteIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select RODAS integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new RODASIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select SEULEX integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new SEULEXIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select Theta time stepping integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new ThetaTimeSteppingIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select Time stepping integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new TimeSteppingIntegrator); });
    integrators->addAction(action);
    action = new QAction("Select Time stepping SSC integrator", this);
    connect(action,&QAction::triggered,this,[=](){ mw->selectSolver(new TimeSteppingSSCIntegrator); });
    integrators->addAction(action);
    addMenu(integrators);
  }

}
