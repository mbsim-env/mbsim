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
#include "view_menu.h"
#include "mainwindow.h"
//#include "element_view.h"
#include "utils.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ViewMenu::ViewMenu(QWidget *parent) : QMenu("View",parent) {
    auto iconPath(mw->getInstallPath()/"share"/"mbsimgui"/"icons");
    auto *action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 0", this);
    action->setShortcut(QKeySequence("0"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(-1); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 1", this);
    action->setShortcut(QKeySequence("1"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(0); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 2", this);
    action->setShortcut(QKeySequence("2"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(1); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 3", this);
    action->setShortcut(QKeySequence("3"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(2); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 4", this);
    action->setShortcut(QKeySequence("4"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(3); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand to depth 5", this);
    action->setShortcut(QKeySequence("5"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(4); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"expanded.svg").string())), "Expand all", this);
    action->setShortcut(QKeySequence("Ctrl++"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(1000); });
    addAction(action);
    action = new QAction(Utils::QIconCached(QString::fromStdString((iconPath/"collapsed.svg").string())), "Collapse all", this);
    action->setShortcut(QKeySequence("Ctrl+-"));
    connect(action,&QAction::triggered,this,[=](){ mw->expandToDepth(-1); });
    addAction(action);
  }

}
