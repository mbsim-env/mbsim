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
#include "integrator.h"
#include "analyzer.h"
#include "solver_view.h"
#include "solver_property_dialog.h"
#include "project.h"
#include "utils.h"
#include "mainwindow.h"
#include <QEvent>

namespace MBSimGUI {

  extern MainWindow *mw;

  SolverViewContextMenu::SolverViewContextMenu(const std::vector<QString> &type, QWidget *parent) : QMenu(parent) {
    auto *action = new QAction(QIcon::fromTheme("document-save-as"), "Save as", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(saveSolverAs()));
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(loadSolver()));
    addAction(action);
    addSeparator();
    QActionGroup *actionGroup = new QActionGroup(this);
    for(size_t i = 0; i<type.size(); i++)
      actionGroup->addAction(new QAction(type[i], this));
    addActions(actionGroup->actions());
    connect(actionGroup,SIGNAL(triggered(QAction*)),this,SLOT(selectSolver(QAction*)));
  }

  void SolverViewContextMenu::selectSolver(QAction *action) {
    QActionGroup *actionGroup = action->actionGroup();
    QList<QAction*> list = actionGroup->actions();
    mw->selectSolver(list.indexOf(action));
  }

  SolverView::SolverView()  {
    type.emplace_back("DOPRI5 integrator");
    type.emplace_back("RADAU5 integrator");
    type.emplace_back("DOP853 integrator");
    type.emplace_back("ODEX integrator");
    type.emplace_back("LSODE integrator");
    type.emplace_back("LSODAR integrator");
    type.emplace_back("LSODER integrator");
    type.emplace_back("Time stepping integrator");
    type.emplace_back("Theta time stepping integrator");
    type.emplace_back("Time stepping SSC integrator");
    type.emplace_back("HETS2 integrator");
    type.emplace_back("Euler explicit integrator");
    type.emplace_back("RKSuite integrator");
    type.emplace_back("Boost odeint DOS RKDOPRI5");
    type.emplace_back("Boost odeint DOS Burlisch Stoer");
    type.emplace_back("Boost odeint DOS Rosenbrock4");
    type.emplace_back("Eigenanalyzer");
    type.emplace_back("Harmonic response analyzer");
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this,SIGNAL(customContextMenuRequested(const QPoint&)),this,SLOT(openContextMenu()));

    installEventFilter(new SolverMouseEvent(this));
    setReadOnly(true);
  }

  void SolverView::openContextMenu() {
    QMenu *menu = createContextMenu();
    menu->exec(QCursor::pos());
    delete menu;
  }

  void SolverView::openEditor() {
    if(!editor) {
      mw->setAllowUndo(false);
      mw->updateParameters(mw->getProject()->getSolver());
      editor = mw->getProject()->getSolver()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
    }
  }

  void SolverView::dialogFinished(int result) {
    if(result != 0) {
      mw->setProjectChanged(true);
      editor->fromWidget();
    }
    editor = nullptr;
    mw->setAllowUndo(true);
  }

  void SolverView::apply() {
    mw->setProjectChanged(true);
    editor->fromWidget();
  }

  Solver* SolverView::createSolver(int i_) {
    i = i_;
    updateText();
    if(i==0)
      return new DOPRI5Integrator;
    else if(i==1)
      return new RADAU5Integrator;
    else if(i==2)
      return new DOP853Integrator;
    else if(i==3)
      return new ODEXIntegrator;
    else if(i==4)
      return new LSODEIntegrator;
    else if(i==5)
      return new LSODARIntegrator;
    else if(i==6)
      return new LSODERIntegrator;
    else if(i==7)
      return new TimeSteppingIntegrator;
    else if(i==8)
      return new ThetaTimeSteppingIntegrator;
    else if(i==9)
      return new TimeSteppingSSCIntegrator;
    else if(i==10)
      return new HETS2Integrator;
    else if(i==11)
      return new EulerExplicitIntegrator;
    else if(i==12)
      return new RKSuiteIntegrator;
    else if(i==13)
      return new BoostOdeintDOS_RKDOPRI5;
    else if(i==14)
      return new BoostOdeintDOS_BulirschStoer;
    else if(i==15)
      return new BoostOdeintDOS_Rosenbrock4;
    else if(i==16)
      return new Eigenanalyzer;
    else if(i==17)
      return new HarmonicResponseAnalyzer;
    return nullptr;
  }

  void SolverView::setSolver(Solver *solver) {
    if(dynamic_cast<DOPRI5Integrator*>(solver))
      i=0;
    else if(dynamic_cast<RADAU5Integrator*>(solver))
      i=1;
    else if(dynamic_cast<DOP853Integrator*>(solver))
      i=2;
    else if(dynamic_cast<ODEXIntegrator*>(solver))
      i=3;
    else if(dynamic_cast<LSODEIntegrator*>(solver))
      i=4;
    else if(dynamic_cast<LSODARIntegrator*>(solver))
      i=5;
    else if(dynamic_cast<LSODERIntegrator*>(solver))
      i=6;
    else if(dynamic_cast<TimeSteppingIntegrator*>(solver))
      i=7;
    else if(dynamic_cast<ThetaTimeSteppingIntegrator*>(solver))
      i=8;
    else if(dynamic_cast<TimeSteppingSSCIntegrator*>(solver))
      i=9;
    else if(dynamic_cast<HETS2Integrator*>(solver))
      i=10;
    else if(dynamic_cast<EulerExplicitIntegrator*>(solver))
      i=11;
    else if(dynamic_cast<RKSuiteIntegrator*>(solver))
      i=12;
    else if(dynamic_cast<BoostOdeintDOS_RKDOPRI5*>(solver))
      i=13;
    else if(dynamic_cast<BoostOdeintDOS_BulirschStoer*>(solver))
      i=14;
    else if(dynamic_cast<BoostOdeintDOS_Rosenbrock4*>(solver))
      i=15;
    else if(dynamic_cast<Eigenanalyzer*>(solver))
      i=16;
    else if(dynamic_cast<HarmonicResponseAnalyzer*>(solver))
      i=17;
    updateText();
  }

  bool SolverMouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonDblClick) {
      view->openEditor();
      return true;
    }
    else if(event->type() == QEvent::MouseButtonPress) {
      mw->solverViewClicked();
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }

}
