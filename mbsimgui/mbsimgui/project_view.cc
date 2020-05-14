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
#include "mainwindow.h"
#include <QEvent>

namespace MBSimGUI {

  extern MainWindow *mw;

  ProjectContextMenu::ProjectContextMenu(QWidget *parent) : QMenu(parent) {
    auto *action=new QAction(QIcon::fromTheme("document-properties"), "Edit", this);
    connect(action,&QAction::triggered,this,[=](){ mw->openProjectEditor(); });
    addAction(action);
    action=new QAction(QIcon::fromTheme("document-properties"), "View XML", this);
    connect(action,&QAction::triggered,mw,&MainWindow::viewProjectSource);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-save-as"), "Export", this);
    connect(action,&QAction::triggered,mw,&MainWindow::saveProjectAs);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-open"), "Import", this);
    connect(action,&QAction::triggered,mw,QOverload<>::of(&MainWindow::loadProject));
    addAction(action);
  }

  bool ProjectMouseEvent::eventFilter(QObject *obj, QEvent *event) {
    if(event->type() == QEvent::MouseButtonDblClick) {
      mw->openProjectEditor();
      return true;
    }
    else if(event->type() == QEvent::MouseButtonPress) {
      mw->projectViewClicked();
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }

}
