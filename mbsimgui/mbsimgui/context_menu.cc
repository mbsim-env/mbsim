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
#include "context_menu.h"
#include "mainwindow.h"
#include "element_view.h"
#include "utils.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ContextMenu::ContextMenu(const QString &title, QWidget *parent) : QMenu(title,parent) {
    auto iconPath(mw->getInstallPath()/"share"/"mbsimgui"/"icons");
    auto *view = new QMenu("View");
    auto *action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expand(mw->getElementView()->selectionModel()->currentIndex()); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 0", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),0); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 1", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),1); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 2", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),2); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 3", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),3); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 4", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),4); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 5", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),5); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand all", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandRecursively(mw->getElementView()->selectionModel()->currentIndex()); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"collapsed.svg").string())), "Collapse", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->collapse(mw->getElementView()->selectionModel()->currentIndex()); });
    view->addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"collapsed.svg").string())), "Collapse all", this);
    connect(action,&QAction::triggered,this,[=](){ mw->getElementView()->expandToDepth(mw->getElementView()->selectionModel()->currentIndex(),-1); });
    view->addAction(action);
    addMenu(view);
    addSeparator();
  }

}
