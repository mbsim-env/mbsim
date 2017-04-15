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
#include "parameter_context_menu.h"
#include "embedding_view.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  ParameterContextMenu::ParameterContextMenu(QWidget *parent) : QMenu(parent) {
    QAction *action=new QAction("Edit", this);
    connect(action,SIGNAL(triggered()),mw->getEmbeddingList(),SLOT(openEditor()));
    addAction(action);
    addSeparator();
    action=new QAction("Copy", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(copyParameter()));
    addAction(action);
    action=new QAction("Cut", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(cutParameter()));
    addAction(action);
    addSeparator();
    action=new QAction("Remove", this);
    connect(action,SIGNAL(triggered()),mw,SLOT(removeParameter()));
    addAction(action);
  }

}
