/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

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
#include "project_view.h"
#include "project.h"
#include "mainwindow.h"
#include <QEvent>

namespace MBSimGUI {

  extern MainWindow *mw;

  ProjectViewContextMenu::ProjectViewContextMenu(QWidget *parent) : QMenu(parent) {
  //  QAction *action=new QAction("Edit", this);
  //  connect(action,SIGNAL(triggered()),mw->getElementList(),SLOT(openEditor()));
  //  addAction(action);
  }

//  void ProjectViewContextMenu::selectProject(QAction *action) {
//    QActionGroup *actionGroup = action->actionGroup();
//    QList<QAction*> list = actionGroup->actions();
//    mw->selectProject(list.indexOf(action));
//  }

  ProjectView::ProjectView() {
    setText("Project");
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this,SIGNAL(customContextMenuRequested(const QPoint&)),this,SLOT(openContextMenu()));
//
    installEventFilter(new ProjectMouseEvent(this));
    setReadOnly(true);
  }

  void ProjectView::updateName() {
    setText(project->getName());
  }

  void ProjectView::openContextMenu() {
    QMenu *menu = createContextMenu();
    menu->exec(QCursor::pos());
    delete menu;
  }

  bool ProjectMouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonDblClick) {
      mw->setAllowUndo(false);
//      mw->updateParameters(view->getProject());
      editor = view->getProject()->createPropertyDialog();
      editor->setAttribute(Qt::WA_DeleteOnClose);
      editor->toWidget();
      editor->show();
      connect(editor,SIGNAL(apply()),this,SLOT(apply()));
      connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
      return true;
    }
    else if(event->type() == QEvent::MouseButtonPress) {
      mw->projectViewClicked();
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }

  void ProjectMouseEvent::dialogFinished(int result) {
    if(result != 0) {
      mw->setProjectChanged(true);
      editor->fromWidget();
      view->updateName();
    }
    editor = nullptr;
    mw->setAllowUndo(true);
  }

  void ProjectMouseEvent::apply() {
    mw->setProjectChanged(true);
    editor->fromWidget();
    view->updateName();
  }

}
