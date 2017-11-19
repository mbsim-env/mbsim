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
#include "embedding_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include "embeditemdata.h"
#include "parameter.h"
#include "mainwindow.h"

namespace MBSimGUI {

  extern MainWindow *mw;

  void EmbeddingView::openEditor() {
    if(!editor) {
      mw->setAllowUndo(false);
      index = selectionModel()->currentIndex();
      parameter = dynamic_cast<Parameter*>(static_cast<EmbeddingTreeModel*>(model())->getItem(index)->getItemData());
      if(parameter) {
        EmbedItemData *item = static_cast<EmbedItemData*>(static_cast<EmbeddingTreeModel*>(model())->getItem(index.parent())->getItemData());
        mw->updateParameters(item,true);
        editor = parameter->createPropertyDialog();
        editor->setAttribute(Qt::WA_DeleteOnClose);
        if(parameter->getConfig())
          editor->toWidget();
        else
          editor->setCancel(false);
        editor->show();
        connect(editor,SIGNAL(apply()),this,SLOT(apply()));
        connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
        return;
      }
      else {
        auto *item = static_cast<EmbedItemData*>(static_cast<EmbeddingTreeModel*>(model())->getItem(index)->getItemData());
        if(item and item->getXMLElement()) {
          mw->updateParameters(item);
          editor = item->createEmbeddingPropertyDialog();
          editor->setAttribute(Qt::WA_DeleteOnClose);
          editor->toWidget();
          editor->show();
          connect(editor,SIGNAL(apply()),this,SLOT(apply()));
          connect(editor,SIGNAL(finished(int)),this,SLOT(dialogFinished(int)));
        }
      }
    }
  }

  void EmbeddingView::mouseDoubleClickEvent(QMouseEvent *event) {
    openEditor();
  }

  void EmbeddingView::mousePressEvent ( QMouseEvent * event ) {
    if(!editor)
      QTreeView::mousePressEvent(event);
  }

  void EmbeddingView::dialogFinished(int result) {
    if(result != 0) {
      if(editor->getCancel())
        mw->setProjectChanged(true);
      editor->fromWidget();
      mw->mbsimxml(1);
      if(parameter)
        parameter->setConfig(true);
    }
    parameter = nullptr;
    editor = nullptr;
    mw->setAllowUndo(true);
  }

  void EmbeddingView::apply() {
    if(editor->getCancel())
      mw->setProjectChanged(true);
    editor->fromWidget();
    update(index);
    update(index.sibling(index.row(),1));
    mw->mbsimxml(1);
    if(parameter)
      parameter->setConfig(true);
    editor->setCancel(true);
  }

}
