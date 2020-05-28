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
#include "parameters_context_menu.h"
#include "mainwindow.h"
#include "parameter.h"
#include "embeditemdata.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParametersContextMenu::ParametersContextMenu(EmbedItemData *item_, const QString &title, QWidget *parent) : QMenu(title,parent), item(item_) {
    QAction *action=new QAction(QIcon::fromTheme("document-properties"), "View XML", this);
    action->setEnabled(item->getNumberOfParameters());
    connect(action,&QAction::triggered,mw,&MainWindow::viewParametersSource);
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("document-save-as"), "Export", this);
    action->setEnabled(item->getNumberOfParameters());
    connect(action,&QAction::triggered,mw,&MainWindow::saveParametersAs);
    addAction(action);
    addSeparator();
    action = new QAction("Embed", this);
    action->setDisabled(item->getNumberOfParameters() or item->hasParameterXMLElement());
    connect(action,&QAction::triggered,this,[=](){ mw->loadParameter(item,nullptr,true); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("document-open"), "Import", this);
    connect(action,&QAction::triggered,this,[=](){ mw->loadParameter(item); });
    addAction(action);
    action = new QAction(QIcon::fromTheme("edit-paste"), "Paste", this);
    action->setEnabled(mw->getParameterBuffer().first);
    connect(action,&QAction::triggered,this,[=](){ mw->loadParameter(item, mw->getParameterBuffer().first); });
    addAction(action);
    addSeparator();
    action = new QAction(QIcon::fromTheme("edit-delete"), "Remove", this);
    action->setEnabled(item->getNumberOfParameters() and not(item->getEmbedItemParent() and item->getEmbedItemParent()->getEmbeded()));
    connect(action,&QAction::triggered,this,[=](){ mw->removeParameter(item); });
    addAction(action);
    addSeparator();
    action = new QAction("Add import parameter", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addParameter(new ImportParameter, item); });
    addAction(action);
    action = new QAction("Add matrix parameter", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addParameter(new MatrixParameter, item); });
    addAction(action);
    action = new QAction("Add scalar parameter", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addParameter(new ScalarParameter, item); });
    addAction(action);
    action = new QAction("Add string parameter", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addParameter(new StringParameter, item); });
    addAction(action);
    action = new QAction("Add vector parameter", this);
    connect(action,&QAction::triggered,this,[=](){ mw->addParameter(new VectorParameter, item); });
    addAction(action);
  }

}
