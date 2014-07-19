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
#include "solver_view.h"
#include "solver_property_dialog.h"
#include "utils.h"
#include "mainwindow.h"
#include <QEvent>

namespace MBSimGUI {

  extern MainWindow *mw;

  IntegratorViewContextMenu::IntegratorViewContextMenu(QWidget *parent) : QMenu(parent) {
    QAction *action = new QAction(Utils::QIconCached("newobject.svg"),"DOPRI5", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectDOPRI5Integrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"RADAU5", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectRADAU5Integrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"LSODE", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectLSODEIntegrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"LSODAR", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectLSODARIntegrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"Time stepping", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectTimeSteppingIntegrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"Euler explicit", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectEulerExplicitIntegrator()));
    addAction(action);
    action = new QAction(Utils::QIconCached("newobject.svg"),"RKSuite", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(selectRKSuiteIntegrator()));
    addAction(action);
  }

  IntegratorView::IntegratorView() : i(0) {
    solver.push_back(new DOPRI5Integrator);
    solver.push_back(new RADAU5Integrator);
    solver.push_back(new LSODEIntegrator);
    solver.push_back(new LSODARIntegrator);
    solver.push_back(new TimeSteppingIntegrator);
    solver.push_back(new EulerExplicitIntegrator);
    solver.push_back(new RKSuiteIntegrator);
    type.push_back("DOPRI5");
    type.push_back("RADAU5");
    type.push_back("LSODE");
    type.push_back("LSODAR");
    type.push_back("Time stepping");
    type.push_back("Euler explicit");
    type.push_back("RKSuite");
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this,SIGNAL(customContextMenuRequested(const QPoint&)),this,SLOT(openContextMenu()));

    installEventFilter(new IntegratorMouseEvent(this));
    setReadOnly(true);
  }

  IntegratorView::~IntegratorView() {
    for(int i=0; i<solver.size(); i++)
      delete solver[i];
  }

  void IntegratorView::openContextMenu() {
    QMenu *menu = createContextMenu();
    menu->exec(QCursor::pos());
    delete menu;
  }

  void IntegratorView::setSolver(Solver *solver_) {
    if(dynamic_cast<DOPRI5Integrator*>(solver_))
      i=0;
    else if(dynamic_cast<RADAU5Integrator*>(solver_))
      i=1;
    else if(dynamic_cast<LSODEIntegrator*>(solver_))
      i=2;
    else if(dynamic_cast<LSODARIntegrator*>(solver_))
      i=3;
    else if(dynamic_cast<TimeSteppingIntegrator*>(solver_))
      i=4;
    else if(dynamic_cast<EulerExplicitIntegrator*>(solver_))
      i=5;
    else if(dynamic_cast<RKSuiteIntegrator*>(solver_))
      i=6;
    delete solver[i];
    solver[i] = solver_;
    updateText();
  }

  bool IntegratorMouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::MouseButtonDblClick) {
      editor = view->getSolver()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
      return true;
    } else
      return QObject::eventFilter(obj, event);
  }

  void IntegratorMouseEvent::dialogFinished(int result) {
    editor = 0;
  }

  void IntegratorMouseEvent::apply() {
  }

}
