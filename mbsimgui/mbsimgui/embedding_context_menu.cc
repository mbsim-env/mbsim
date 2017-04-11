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
#include "embedding_context_menu.h"
#include "mainwindow.h"
#include "parameter.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  EmbeddingContextMenu::EmbeddingContextMenu(EmbedItemData *item_, const QString &title, QWidget *parent) : QMenu(title,parent), item(item_) {
    QAction *action = new QAction("Add scalar parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addScalarParameter()));
    addAction(action);
    action = new QAction("Add vector parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addVectorParameter()));
    addAction(action);
    action = new QAction("Add matrix parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addMatrixParameter()));
    addAction(action);
    action = new QAction("Add string parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addStringParameter()));
    addAction(action);
    action = new QAction("Add import parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addImportParameter()));
    addAction(action);
  }

  void EmbeddingContextMenu::addScalarParameter() {
    mw->addParameter(new ScalarParameter("a"), item);
  }

  void EmbeddingContextMenu::addVectorParameter() {
    mw->addParameter(new VectorParameter("a"), item);
  }

  void EmbeddingContextMenu::addMatrixParameter() {
    mw->addParameter(new MatrixParameter("a"), item);
  }

  void EmbeddingContextMenu::addStringParameter() {
    mw->addParameter(new StringParameter("a"), item);
  }

  void EmbeddingContextMenu::addImportParameter() {
    mw->addParameter(new ImportParameter, item);
  }

}
