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
#include "analyser.h"
#include "solver_view.h"
#include "solver_property_dialog.h"
#include "project.h"
#include "utils.h"
#include "mainwindow.h"
#include <QEvent>

namespace MBSimGUI {

  extern MainWindow *mw;

  SolverViewContextMenu::SolverViewContextMenu(QWidget *parent) : QMenu(parent) {
    auto *action = new QAction(QIcon::fromTheme("document-save-as"), "Save as", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(saveSolverAs()));
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Load", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(loadSolver()));
    addAction(action);
    addSeparator();
    QActionGroup *actionGroup = new QActionGroup(this);
    action = new QAction("DOPRI5 integrator", this);
    actionGroup->addAction(action);
    action = new QAction("RADAU5 integrator", this);
    actionGroup->addAction(action);
    action = new QAction("LSODE integrator", this);
    actionGroup->addAction(action);
    action = new QAction("LSODAR integrator", this);
    actionGroup->addAction(action);
    action = new QAction("Time stepping integrator", this);
    actionGroup->addAction(action);
    action = new QAction("Euler explicit integrator", this);
    actionGroup->addAction(action);
    action = new QAction("RKSuite integrator", this);
    actionGroup->addAction(action);
    action = new QAction("Eigenanalyser", this);
    actionGroup->addAction(action);
    action = new QAction("HarmonicResponseAnalyser", this);
    actionGroup->addAction(action);
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
    type.emplace_back("LSODE integrator");
    type.emplace_back("LSODAR integrator");
    type.emplace_back("Time stepping integrator");
    type.emplace_back("Euler explicit integrator");
    type.emplace_back("RKSuite integrator");
    type.emplace_back("Eigenanalyser");
    type.emplace_back("HarmonicResponseAnalyser");
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

  Solver* SolverView::createSolver(int i_) {
    i = i_;
    updateText();
    if(i==0)
      return new DOPRI5Integrator;
    else if(i==1)
      return new RADAU5Integrator;
    else if(i==2)
      return new LSODEIntegrator;
    else if(i==3)
      return new LSODARIntegrator;
    else if(i==4)
      return new TimeSteppingIntegrator;
    else if(i==5)
      return new EulerExplicitIntegrator;
    else if(i==6)
      return new RKSuiteIntegrator;
    else if(i==7)
      return new Eigenanalyser;
    else if(i==8)
      return new HarmonicResponseAnalyser;
    return nullptr;
  }

  void SolverView::setSolver(Solver *solver) {
    if(dynamic_cast<DOPRI5Integrator*>(solver))
      i=0;
    else if(dynamic_cast<RADAU5Integrator*>(solver))
      i=1;
    else if(dynamic_cast<LSODEIntegrator*>(solver))
      i=2;
    else if(dynamic_cast<LSODARIntegrator*>(solver))
      i=3;
    else if(dynamic_cast<TimeSteppingIntegrator*>(solver))
      i=4;
    else if(dynamic_cast<EulerExplicitIntegrator*>(solver))
      i=5;
    else if(dynamic_cast<RKSuiteIntegrator*>(solver))
      i=6;
    else if(dynamic_cast<Eigenanalyser*>(solver))
      i=7;
    else if(dynamic_cast<HarmonicResponseAnalyser*>(solver))
      i=8;
    updateText();
  }

  bool SolverMouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonDblClick) {
      mw->setAllowUndo(false);
      mw->updateParameters(mw->getProject()->getSolver());
      editor = mw->getProject()->getSolver()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
      return true;
    }
    else if(event->type() == QEvent::MouseButtonPress) {
      mw->solverViewClicked();
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }

  void SolverMouseEvent::dialogFinished(int result) {
    if(result != 0) {
      mw->setProjectChanged(true);
      editor->fromWidget();
    }
    editor = nullptr;
    mw->setAllowUndo(true);
  }

  void SolverMouseEvent::apply() {
    mw->setProjectChanged(true);
    editor->fromWidget();
  }

}
